#include "galaxy_camera/galaxy_camera.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>

namespace galaxy_camera {

namespace {
std::atomic<int> g_sdk_ref{0};
std::mutex g_sdk_mutex;

bool is_usb_device_type(int dev_type) {
    return dev_type == GX_DEVICE_CLASS_U3V || dev_type == GX_DEVICE_CLASS_USB2;
}

bool gx_ok(GX_STATUS s, const char* what) {
    if (s == GX_STATUS_SUCCESS) {
        return true;
    }
    std::cerr << what << " failed, status=" << static_cast<int>(s) << "\n";
    return false;
}

bool gx_optional_set_ok(GX_STATUS s, const char* what) {
    if (s == GX_STATUS_SUCCESS) {
        return true;
    }
    if (s == GX_STATUS_INVALID_ACCESS || s == GX_STATUS_NOT_IMPLEMENTED) {
        std::cerr << what << " skipped, status=" << static_cast<int>(s) << "\n";
        return true;
    }
    std::cerr << what << " failed, status=" << static_cast<int>(s) << "\n";
    return false;
}

bool set_enum_value_optional(GX_DEV_HANDLE h, const char* name, int64_t value) {
    GX_ENUM_VALUE enum_value{};
    if (GXGetEnumValue(h, name, &enum_value) != GX_STATUS_SUCCESS) {
        return true;
    }
    return gx_optional_set_ok(GXSetEnumValue(h, name, value), name);
}

bool set_bool_optional(GX_DEV_HANDLE h, const char* name, bool value) {
    bool cur = false;
    if (GXGetBoolValue(h, name, &cur) != GX_STATUS_SUCCESS) {
        return true;
    }
    return gx_optional_set_ok(GXSetBoolValue(h, name, value), name);
}

bool set_float_clamped_optional(GX_DEV_HANDLE h, const char* name, double value) {
    GX_FLOAT_VALUE range{};
    if (GXGetFloatValue(h, name, &range) != GX_STATUS_SUCCESS) {
        return true;
    }
    const double clamped = std::max(range.dMin, std::min(value, range.dMax));
    return gx_optional_set_ok(GXSetFloatValue(h, name, clamped), name);
}

bool set_int_clamped_optional(GX_DEV_HANDLE h, const char* name, int64_t value) {
    GX_INT_VALUE range{};
    if (GXGetIntValue(h, name, &range) != GX_STATUS_SUCCESS) {
        return true;
    }
    int64_t clamped = std::max(range.nMin, std::min(value, range.nMax));
    if (range.nInc > 1) {
        clamped = range.nMin + ((clamped - range.nMin) / range.nInc) * range.nInc;
    }
    return gx_optional_set_ok(GXSetIntValue(h, name, clamped), name);
}

}  // namespace

GalaxyCamera::GalaxyCamera() = default;

GalaxyCamera::~GalaxyCamera() {
    close();
}

bool GalaxyCamera::initSdkOnce() {
    std::lock_guard<std::mutex> lock(g_sdk_mutex);
    if (g_sdk_ref.fetch_add(1) == 0) {
        if (!gx_ok(GXInitLib(), "GXInitLib")) {
            g_sdk_ref.fetch_sub(1);
            return false;
        }
    }
    return true;
}

void GalaxyCamera::releaseSdkOnce() {
    std::lock_guard<std::mutex> lock(g_sdk_mutex);
    if (g_sdk_ref.fetch_sub(1) == 1) {
        GXCloseLib();
    }
}

bool GalaxyCamera::open(const CameraConfig& config) {
    if (handle_) {
        return true;
    }
    if (!initSdkOnce()) {
        return false;
    }
    config_ = config;
    if (!reopenDevice()) {
        releaseSdkOnce();
        return false;
    }
    return true;
}

void GalaxyCamera::close() {
    if (handle_ && config_.feature_save_enable && config_.feature_save_on_close &&
        !config_.feature_save_path.empty()) {
        applyFeatureSave(config_.feature_save_path);
    }
    stopGrabbing();
    closeDevice();
    releaseGpuResources();
    exception_registered_ = false;
    if (g_sdk_ref.load() > 0) {
        releaseSdkOnce();
    }
}

bool GalaxyCamera::openDevice() {
    return handle_ != nullptr;
}

void GalaxyCamera::closeDevice() {
    if (handle_) {
        GXCloseDevice(handle_);
        handle_ = nullptr;
    }
}

bool GalaxyCamera::applyConfig() {
    if (!handle_) {
        return false;
    }

    bool ok = true;
    ok &= gx_ok(GXSetEnumValueByString(handle_, "AcquisitionMode", "Continuous"), "AcquisitionMode");
    ok &= gx_ok(GXSetEnumValueByString(handle_, "TriggerMode", config_.trigger_mode ? "On" : "Off"), "TriggerMode");
    if (config_.trigger_mode) {
        ok &= set_enum_value_optional(handle_, "TriggerSource", config_.trigger_source);
    }

    ok &= set_enum_value_optional(handle_, "ExposureAuto", config_.auto_exposure ? 2 : 0);
    if (!config_.auto_exposure) {
        ok &= set_float_clamped_optional(handle_, "ExposureTime", config_.exposure_time_us);
    }

    ok &= set_enum_value_optional(handle_, "GainAuto", config_.auto_gain ? 2 : 0);
    if (!config_.auto_gain) {
        ok &= set_float_clamped_optional(handle_, "Gain", config_.gain);
    }

    ok &= set_enum_value_optional(handle_, "BalanceWhiteAuto", config_.balance_white_auto ? 2 : 0);

    ok &= set_bool_optional(handle_, "AcquisitionFrameRateMode", config_.frame_rate_enable);
    ok &= set_bool_optional(handle_, "AcquisitionFrameRateEnable", config_.frame_rate_enable);
    if (config_.frame_rate_enable) {
        ok &= set_float_clamped_optional(handle_, "AcquisitionFrameRate", config_.frame_rate);
    }

    // 不在启动阶段强制写 PixelFormat，避免部分机型节点只读导致无意义报错。
    // 颜色转换按每帧上报的 pixel_type 自动处理。
    ok &= setRoi(config_.roi);

    return ok;
}

bool GalaxyCamera::applyFeatureLoad() {
    if (!handle_ || !config_.feature_load_enable || config_.feature_load_path.empty()) {
        return true;
    }
    return gx_ok(GXFeatureLoad(handle_, config_.feature_load_path.c_str(), true), "GXFeatureLoad");
}

bool GalaxyCamera::applyFeatureSave(const std::string& path) {
    if (!handle_ || path.empty()) {
        return false;
    }
    std::filesystem::path save_path(path);
    if (save_path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(save_path.parent_path(), ec);
    }
    return gx_ok(GXFeatureSave(handle_, path.c_str()), "GXFeatureSave");
}

void GalaxyCamera::registerExceptionCallback() {
    exception_registered_ = true;
}

void GalaxyCamera::handleDisconnect() {
    if (!config_.reconnect_enable) {
        return;
    }
    std::lock_guard<std::mutex> lock(reconnect_mutex_);
    if (reconnecting_) {
        return;
    }
    reconnecting_ = true;
    stopGrabbing();
    reopenDevice();
    reconnecting_ = false;
}

void GX_STDC GalaxyCamera::ExceptionCallback(unsigned int, void*) {
}

bool GalaxyCamera::reopenDevice() {
    closeDevice();

    uint32_t device_num = 0;
    if (!gx_ok(GXUpdateAllDeviceList(&device_num, 1000), "GXUpdateAllDeviceList") || device_num == 0) {
        std::cerr << "No Galaxy camera found\n";
        return false;
    }

    std::vector<uint32_t> usb_indices;
    usb_indices.reserve(device_num);
    for (uint32_t i = 1; i <= device_num; ++i) {
        GX_DEVICE_INFO info{};
        if (GXGetDeviceInfo(i, &info) != GX_STATUS_SUCCESS) {
            continue;
        }
        if (is_usb_device_type(info.emDevType)) {
            usb_indices.push_back(i);
        }
    }
    if (usb_indices.empty()) {
        std::cerr << "No USB Galaxy camera found (U3V/USB2)\n";
        return false;
    }

    uint32_t selected_index = usb_indices.front();
    if (!config_.serial_number.empty()) {
        bool found = false;
        for (uint32_t i : usb_indices) {
            GX_DEVICE_INFO info{};
            if (GXGetDeviceInfo(i, &info) != GX_STATUS_SUCCESS) {
                continue;
            }
            if (getDeviceSerial(info) == config_.serial_number) {
                selected_index = i;
                found = true;
                break;
            }
        }
        if (!found) {
            std::cerr << "Serial not found: " << config_.serial_number << "\n";
            return false;
        }
    } else if (!config_.use_first_device) {
        const int wanted = std::max(0, config_.device_index);
        const int clamped = std::min(wanted, static_cast<int>(usb_indices.size()) - 1);
        selected_index = usb_indices[static_cast<size_t>(clamped)];
    }

    GX_DEVICE_INFO selected_info{};
    if (!gx_ok(GXGetDeviceInfo(selected_index, &selected_info), "GXGetDeviceInfo")) {
        return false;
    }
    printDeviceInfo(selected_info);

    std::string index_str = std::to_string(selected_index);
    GX_OPEN_PARAM open_param{};
    open_param.openMode = GX_OPEN_INDEX;
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    open_param.pszContent = const_cast<char*>(index_str.c_str());

    if (!gx_ok(GXOpenDevice(&open_param, &handle_), "GXOpenDevice")) {
        handle_ = nullptr;
        return false;
    }

    registerExceptionCallback();
    if (!applyFeatureLoad()) {
        closeDevice();
        return false;
    }
    if (!applyConfig()) {
        closeDevice();
        return false;
    }
    if (!prepareBuffers()) {
        closeDevice();
        return false;
    }
    return true;
}

bool GalaxyCamera::startGrabbing() {
    if (!handle_) {
        return false;
    }
    if (grabbing_) {
        return true;
    }
    if (!gx_ok(GXStreamOn(handle_), "GXStreamOn")) {
        return false;
    }
    grabbing_ = true;
    exit_thread_ = false;
    if (callback_) {
        grab_thread_ = std::thread(&GalaxyCamera::grabLoop, this);
    }
    return true;
}

void GalaxyCamera::stopGrabbing() {
    exit_thread_ = true;
    if (grab_thread_.joinable()) {
        grab_thread_.join();
    }
    if (handle_ && grabbing_) {
        GXStreamOff(handle_);
    }
    grabbing_ = false;
}

bool GalaxyCamera::read(Frame* out, int timeout_ms) {
    if (!handle_ || !out) {
        return false;
    }
    if (callback_ && grab_thread_.joinable()) {
        return false;
    }
    if (!grabbing_ && !startGrabbing()) {
        return false;
    }
    if (grabOnce(out, timeout_ms)) {
        return true;
    }
    if (!config_.reconnect_enable) {
        return false;
    }
    if (!reopenDevice()) {
        return false;
    }
    return startGrabbing() && grabOnce(out, timeout_ms);
}

void GalaxyCamera::setFrameCallback(FrameCallback cb) {
    callback_ = std::move(cb);
    if (callback_ && grabbing_ && !grab_thread_.joinable()) {
        exit_thread_ = false;
        grab_thread_ = std::thread(&GalaxyCamera::grabLoop, this);
    }
}

bool GalaxyCamera::isOpen() const {
    return handle_ != nullptr;
}

bool GalaxyCamera::isGrabbing() const {
    return grabbing_;
}

bool GalaxyCamera::setExposure(double us) {
    if (!handle_) {
        return false;
    }
    config_.auto_exposure = false;
    return set_enum_value_optional(handle_, "ExposureAuto", 0) &&
           set_float_clamped_optional(handle_, "ExposureTime", us);
}

bool GalaxyCamera::setGain(double gain) {
    if (!handle_) {
        return false;
    }
    config_.auto_gain = false;
    return set_enum_value_optional(handle_, "GainAuto", 0) &&
           set_float_clamped_optional(handle_, "Gain", gain);
}

bool GalaxyCamera::setFrameRate(double fps) {
    if (!handle_) {
        return false;
    }
    config_.frame_rate_enable = true;
    return set_bool_optional(handle_, "AcquisitionFrameRateEnable", true) &&
           set_float_clamped_optional(handle_, "AcquisitionFrameRate", fps);
}

bool GalaxyCamera::setTriggerMode(bool enable) {
    if (!handle_) {
        return false;
    }
    config_.trigger_mode = enable;
    return gx_ok(GXSetEnumValueByString(handle_, "TriggerMode", enable ? "On" : "Off"), "TriggerMode");
}

bool GalaxyCamera::setRoi(const RoiConfig& roi) {
    if (!handle_ || grabbing_) {
        return false;
    }

    if (!roi.enable) {
        bool ok = true;
        GX_INT_VALUE w{};
        GX_INT_VALUE h{};
        if (GXGetIntValue(handle_, "Width", &w) == GX_STATUS_SUCCESS) {
            ok &= set_int_clamped_optional(handle_, "Width", w.nMax);
        }
        if (GXGetIntValue(handle_, "Height", &h) == GX_STATUS_SUCCESS) {
            ok &= set_int_clamped_optional(handle_, "Height", h.nMax);
        }
        ok &= set_int_clamped_optional(handle_, "OffsetX", 0);
        ok &= set_int_clamped_optional(handle_, "OffsetY", 0);
        if (ok) {
            config_.roi = roi;
            ok &= prepareBuffers();
        }
        return ok;
    }

    bool ok = true;
    ok &= set_int_clamped_optional(handle_, "Width", roi.width > 0 ? roi.width : 0);
    ok &= set_int_clamped_optional(handle_, "Height", roi.height > 0 ? roi.height : 0);
    ok &= set_int_clamped_optional(handle_, "OffsetX", roi.offset_x);
    ok &= set_int_clamped_optional(handle_, "OffsetY", roi.offset_y);
    if (ok) {
        config_.roi = roi;
        ok &= prepareBuffers();
    }
    return ok;
}

const CameraConfig& GalaxyCamera::config() const {
    return config_;
}

std::string GalaxyCamera::getDeviceSerial(const GX_DEVICE_INFO& info) {
    switch (info.emDevType) {
        case GX_DEVICE_CLASS_U3V:
            return reinterpret_cast<const char*>(info.DevInfo.stU3VDevInfo.chSerialNumber);
        case GX_DEVICE_CLASS_USB2:
            return reinterpret_cast<const char*>(info.DevInfo.stUSBDevInfo.chSerialNumber);
        default:
            return {};
    }
}

void GalaxyCamera::printDeviceInfo(const GX_DEVICE_INFO& info) {
    const char* sn = "";
    const char* model = "";
    switch (info.emDevType) {
        case GX_DEVICE_CLASS_U3V:
            sn = reinterpret_cast<const char*>(info.DevInfo.stU3VDevInfo.chSerialNumber);
            model = reinterpret_cast<const char*>(info.DevInfo.stU3VDevInfo.chModelName);
            break;
        case GX_DEVICE_CLASS_USB2:
            sn = reinterpret_cast<const char*>(info.DevInfo.stUSBDevInfo.chSerialNumber);
            model = reinterpret_cast<const char*>(info.DevInfo.stUSBDevInfo.chModelName);
            break;
        default:
            break;
    }
    std::cout << "Galaxy camera: model=" << model << " sn=" << sn << "\n";
}

}  // namespace galaxy_camera

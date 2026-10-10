// Adapted from JetPilot tools/silkyevcam_bias_tuner (2026-10-10).
#include <metavision/sdk/stream/camera.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/utils/device_config.h>
#include <metavision/sdk/base/events/event_cd.h>

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <filesystem>

// ============================================================
// Bias GUI control
// ============================================================
struct BiasControl {
    std::string name;
    int min_value;
    int max_value;
    int value;
};

// ============================================================
// Auto tuning state
// ============================================================
enum class AutoTuneState {
    IDLE,
    WAITING,
    MEASURING,
    FINISHED,
    FAILED
};

int main(int argc, char **argv) {
    try {

        std::string serial, bias_file, output_dir = ".";
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help" || arg == "-h") {
                std::cout << "Usage: silkyevcam_bias_tuner [--serial SERIAL] "
                          << "[--bias-file FILE.bias] [--output-dir DIR]\n";
                return 0;
            }
            if (arg != "--serial" && arg != "--bias-file" && arg != "--output-dir") {
                throw std::runtime_error("Unknown option: " + arg);
            }
            if (++i >= argc) throw std::runtime_error("Missing value for " + arg);
            if (arg == "--serial") serial = argv[i];
            if (arg == "--bias-file") bias_file = argv[i];
            if (arg == "--output-dir") output_dir = argv[i];
        }
        if (!bias_file.empty()) bias_file = std::filesystem::absolute(bias_file).string();
        std::filesystem::create_directories(output_dir);
        std::filesystem::current_path(output_dir);

        // ============================================================
        // Camera
        // ============================================================
        Metavision::DeviceConfig device_config;
        Metavision::Camera camera =
            serial.empty() ? Metavision::Camera::from_first_available(device_config) :
            Metavision::Camera::from_serial(serial, device_config);

        const auto &geometry = camera.geometry();

        const int width  = geometry.get_width();
        const int height = geometry.get_height();

        std::cout << "Camera opened: "
                  << width << " x " << height
                  << std::endl;

        // ============================================================
        // Bias interface
        // ============================================================
        auto &biases =
            camera.get_facility<Metavision::I_LL_Biases>();

        if (!bias_file.empty()) biases.load_from_file(bias_file);
        const auto startup_biases = biases.get_all_biases();

        std::cout << "\nAvailable biases\n";
        std::cout << "----------------------------\n";

        for (const auto &[name, value] : startup_biases) {
            std::cout
                << name
                << " = "
                << value
                << std::endl;
        }

        // Keep a persistent copy as well as the in-memory snapshot. For VGA
        // sensors these are absolute values, so restoring every bias to zero
        // would not restore the camera's startup state.
        try {
            biases.save_to_file(
                "silkyevcam_startup.bias"
            );

            std::cout
                << "Saved startup biases: "
                << "silkyevcam_startup.bias"
                << std::endl;

        } catch (const std::exception &e) {
            // Reset still works from the in-memory snapshot when the current
            // directory is not writable.
            std::cerr
                << "Could not save startup biases: "
                << e.what()
                << std::endl;
        }

        // ============================================================
        // Bias controls
        // ============================================================
        std::vector<BiasControl> controls;

        const std::vector<std::string> target_biases = {
            "bias_diff_on",
            "bias_diff_off",
            "bias_refr",
            "bias_fo",
            "bias_hpf",
            "bias_diff"
        };

        for (const auto &name : target_biases) {

            if (startup_biases.find(name) == startup_biases.end()) {
                std::cout
                    << "Bias not available: "
                    << name
                    << std::endl;
                continue;
            }

            const int value =
                biases.get(name);

            BiasControl ctrl;
            ctrl.name      = name;
            ctrl.value     = value;
            ctrl.min_value = 0;
            ctrl.max_value = 2000;

            controls.push_back(ctrl);
        }

        // ============================================================
        // Find auto-tune biases
        // ============================================================
        BiasControl *bias_diff_on_ctrl  = nullptr;
        BiasControl *bias_diff_off_ctrl = nullptr;

        for (auto &ctrl : controls) {
            if (ctrl.name == "bias_diff_on") {
                bias_diff_on_ctrl = &ctrl;
            }

            if (ctrl.name == "bias_diff_off") {
                bias_diff_off_ctrl = &ctrl;
            }
        }

        if (!bias_diff_on_ctrl ||
            !bias_diff_off_ctrl) {

            std::cerr
                << "bias_diff_on/off not available."
                << std::endl;

            return 1;
        }

        // ============================================================
        // Visualization
        // ============================================================
        cv::Mat event_image(
            height,
            width,
            CV_8UC3,
            cv::Scalar(255, 255, 255)
        );

        std::mutex image_mutex;

        std::atomic<uint64_t> event_counter{0};
        std::atomic<uint64_t> on_counter{0};
        std::atomic<uint64_t> off_counter{0};

        // ============================================================
        // Event callback
        // ============================================================
        camera.cd().add_callback(
            [&](const Metavision::EventCD *begin,
                const Metavision::EventCD *end) {

                std::lock_guard<std::mutex>
                    lock(image_mutex);

                for (auto it = begin; it != end; ++it) {

                    const auto &ev = *it;

                    if (ev.x >= width ||
                        ev.y >= height) {
                        continue;
                    }

                    if (ev.p) {

                        // Kart EVS palette: ON = red, OFF = blue,
                        // no event = white. OpenCV stores colors as BGR.
                        event_image.at<cv::Vec3b>(
                            ev.y,
                            ev.x
                        ) = cv::Vec3b(
                            0,
                            0,
                            255
                        );

                        on_counter.fetch_add(
                            1,
                            std::memory_order_relaxed
                        );

                    } else {

                        // OFF event = blue
                        event_image.at<cv::Vec3b>(
                            ev.y,
                            ev.x
                        ) = cv::Vec3b(
                            255,
                            0,
                            0
                        );

                        off_counter.fetch_add(
                            1,
                            std::memory_order_relaxed
                        );
                    }

                    event_counter.fetch_add(
                        1,
                        std::memory_order_relaxed
                    );
                }
            }
        );

        // ============================================================
        // GUI
        // ============================================================
        const std::string window_name =
            "SilkyEvCam Bias Tuner";

        cv::namedWindow(
            window_name,
            cv::WINDOW_NORMAL
        );

        cv::resizeWindow(
            window_name,
            std::max(900, width),
            std::max(700, height)
        );

        for (auto &ctrl : controls) {

            cv::createTrackbar(
                ctrl.name,
                window_name,
                nullptr,
                ctrl.max_value
            );

            cv::setTrackbarPos(
                ctrl.name,
                window_name,
                ctrl.value
            );
        }

        // Prefer the native Qt push button when available. OpenCV builds that
        // use GTK instead of Qt do not implement createButton(), so provide a
        // small clickable fallback window rather than exposing a 0/1 slider.
        std::atomic<bool> reset_requested{false};

        const auto reset_button_callback =
            [](int, void *userdata) {
                auto *requested =
                    static_cast<std::atomic<bool> *>(userdata);

                requested->store(
                    true,
                    std::memory_order_relaxed
                );
            };

        bool using_fallback_reset_button = false;
        const std::string controls_window_name =
            "SilkyEvCam Controls";

        try {
            cv::createButton(
                "Restore startup biases",
                reset_button_callback,
                &reset_requested,
                cv::QT_PUSH_BUTTON,
                false
            );

        } catch (const cv::Exception &) {
            using_fallback_reset_button = true;

            cv::namedWindow(
                controls_window_name,
                cv::WINDOW_AUTOSIZE
            );

            cv::setMouseCallback(
                controls_window_name,
                [](int event,
                   int x,
                   int y,
                   int,
                   void *userdata) {
                    if (event != cv::EVENT_LBUTTONUP) {
                        return;
                    }

                    const cv::Rect button_bounds(
                        10,
                        10,
                        300,
                        48
                    );

                    if (!button_bounds.contains(
                            cv::Point(x, y)
                        )) {
                        return;
                    }

                    auto *requested =
                        static_cast<std::atomic<bool> *>(userdata);

                    requested->store(
                        true,
                        std::memory_order_relaxed
                    );
                },
                &reset_requested
            );
        }

        // ============================================================
        // Start camera
        // ============================================================
        camera.start();

        std::cout << "\nControls\n";
        std::cout << "----------------------------\n";
        std::cout << "a : Auto Tune background noise\n";
        std::cout << "s : Save biases\n";
        std::cout << "r : Reload biases\n";
        std::cout << "x : Restore startup biases\n";
        std::cout << "GUI: click Restore startup biases\n";
        std::cout << "q : Quit\n";
        std::cout << std::endl;

        // ============================================================
        // Normal event-rate measurement
        // ============================================================
        auto previous_rate_time =
            std::chrono::steady_clock::now();

        uint64_t previous_events = 0;
        uint64_t previous_on     = 0;
        uint64_t previous_off    = 0;

        double event_rate = 0.0;
        double on_rate    = 0.0;
        double off_rate   = 0.0;

        // ============================================================
        // Auto tuning parameters
        // ============================================================

        //
        // Background noise target
        //
        // 0.05 Mev/s = 50,000 events/s
        //
        const double target_on_rate =
            50000.0;

        const double target_off_rate =
            50000.0;

        //
        // Bias adjustment step
        //
        const int tune_step = 5;

        //
        // Measurement duration
        //
        const double measurement_seconds =
            5.0;

        //
        // Wait after changing biases
        //
        const double settling_seconds =
            1.0;

        //
        // Maximum number of tuning attempts
        //
        const int max_tune_iterations =
            20;

        AutoTuneState tune_state =
            AutoTuneState::IDLE;

        int tune_iteration = 0;

        auto tune_state_start =
            std::chrono::steady_clock::now();

        uint64_t tune_start_on  = 0;
        uint64_t tune_start_off = 0;

        double measured_on_rate  = 0.0;
        double measured_off_rate = 0.0;

        std::string tune_message =
            "Auto Tune: idle";

        // ============================================================
        // Restore the exact values captured before camera.start().
        // ============================================================
        const auto restore_startup_biases = [&]() {
            bool success = true;
            auto pending_biases = startup_biases;
            std::map<std::string, std::string> restore_errors;

            std::cout
                << "\nRestoring startup biases..."
                << std::endl;

            // Some VGA biases have constraints relative to other biases. A
            // value can therefore be rejected temporarily depending on the
            // restore order. Retry pending values after every successful pass
            // instead of treating the first rejection as a permanent error.
            for (std::size_t pass = 0;
                 pass <= startup_biases.size() && !pending_biases.empty();
                 ++pass) {

                bool made_progress = false;

                for (auto it = pending_biases.begin();
                     it != pending_biases.end();) {

                    const std::string name = it->first;
                    const int startup_value = it->second;

                    try {
                        if (!biases.set(name, startup_value)) {
                            throw std::runtime_error(
                                "set() returned false"
                            );
                        }

                        const int restored_value =
                            biases.get(name);

                        if (restored_value != startup_value) {
                            throw std::runtime_error(
                                "hardware returned " +
                                std::to_string(restored_value) +
                                " instead of " +
                                std::to_string(startup_value)
                            );
                        }

                        std::cout
                            << "  "
                            << name
                            << " = "
                            << restored_value
                            << std::endl;

                        restore_errors.erase(name);
                        it = pending_biases.erase(it);
                        made_progress = true;

                    } catch (const std::exception &e) {
                        restore_errors[name] = e.what();
                        ++it;
                    }
                }

                if (!made_progress) {
                    break;
                }
            }

            if (!pending_biases.empty()) {
                success = false;

                for (const auto &[name, startup_value] : pending_biases) {
                    const auto error = restore_errors.find(name);

                    std::cerr
                        << "Failed to restore "
                        << name
                        << " = "
                        << startup_value
                        << ": "
                        << (error != restore_errors.end() ?
                            error->second :
                            "unknown error")
                        << std::endl;
                }
            }

            // Always re-read the hardware so that a partial failure is shown
            // accurately in the GUI rather than leaving stale slider values.
            for (auto &ctrl : controls) {
                try {
                    ctrl.value = biases.get(ctrl.name);

                    cv::setTrackbarPos(
                        ctrl.name,
                        window_name,
                        ctrl.value
                    );

                } catch (const std::exception &e) {
                    success = false;

                    std::cerr
                        << "Failed to refresh "
                        << ctrl.name
                        << ": "
                        << e.what()
                        << std::endl;
                }
            }

            tune_state = AutoTuneState::IDLE;
            tune_iteration = 0;
            tune_message = success ?
                "Auto Tune: idle | startup biases restored" :
                "Auto Tune: idle | startup restore FAILED";

            std::cout
                << (success ?
                    "Startup biases restored." :
                    "Startup bias restore completed with errors.")
                << std::endl;

            return success;
        };

        // ============================================================
        // Main loop
        // ============================================================
        while (camera.is_running()) {

            // Handle GUI reset requests before manual slider updates so the
            // restored values are not immediately overwritten by old GUI
            // positions.
            if (reset_requested.exchange(
                    false,
                    std::memory_order_relaxed
                )) {

                restore_startup_biases();
            }

            // ========================================================
            // Manual bias updates
            //
            // Disabled while auto-tune is active
            // ========================================================
            if (tune_state == AutoTuneState::IDLE ||
                tune_state == AutoTuneState::FINISHED ||
                tune_state == AutoTuneState::FAILED) {

                for (auto &ctrl : controls) {

                    int gui_value =
                        cv::getTrackbarPos(
                            ctrl.name,
                            window_name
                        );

                    if (gui_value ==
                        ctrl.value) {
                        continue;
                    }

                    try {

                        const bool success =
                            biases.set(
                                ctrl.name,
                                gui_value
                            );

                        if (!success) {
                            throw std::runtime_error(
                                "set() returned false"
                            );
                        }

                        ctrl.value =
                            biases.get(
                                ctrl.name
                            );

                        if (ctrl.value !=
                            gui_value) {

                            cv::setTrackbarPos(
                                ctrl.name,
                                window_name,
                                ctrl.value
                            );
                        }

                        std::cout
                            << ctrl.name
                            << " = "
                            << ctrl.value
                            << std::endl;

                    } catch (
                        const std::exception &e
                    ) {

                        std::cerr
                            << "Failed to set "
                            << ctrl.name
                            << ": "
                            << e.what()
                            << std::endl;

                        cv::setTrackbarPos(
                            ctrl.name,
                            window_name,
                            ctrl.value
                        );
                    }
                }
            }

            // ========================================================
            // Normal rate measurement
            // ========================================================
            auto now =
                std::chrono::steady_clock::now();

            const double dt =
                std::chrono::duration<double>(
                    now -
                    previous_rate_time
                ).count();

            if (dt >= 0.5) {

                const uint64_t current_events =
                    event_counter.load(
                        std::memory_order_relaxed
                    );

                const uint64_t current_on =
                    on_counter.load(
                        std::memory_order_relaxed
                    );

                const uint64_t current_off =
                    off_counter.load(
                        std::memory_order_relaxed
                    );

                event_rate =
                    static_cast<double>(
                        current_events -
                        previous_events
                    ) / dt;

                on_rate =
                    static_cast<double>(
                        current_on -
                        previous_on
                    ) / dt;

                off_rate =
                    static_cast<double>(
                        current_off -
                        previous_off
                    ) / dt;

                previous_events =
                    current_events;

                previous_on =
                    current_on;

                previous_off =
                    current_off;

                previous_rate_time =
                    now;
            }

            // ========================================================
            // AUTO TUNE
            // ========================================================

            // --------------------------------------------------------
            // WAITING
            // --------------------------------------------------------
            if (tune_state ==
                AutoTuneState::WAITING) {

                const double elapsed =
                    std::chrono::duration<double>(
                        now -
                        tune_state_start
                    ).count();

                char buffer[256];

                std::snprintf(
                    buffer,
                    sizeof(buffer),
                    "Auto Tune: settling %.1f / %.1f sec",
                    elapsed,
                    settling_seconds
                );

                tune_message =
                    buffer;

                if (elapsed >=
                    settling_seconds) {

                    tune_start_on =
                        on_counter.load(
                            std::memory_order_relaxed
                        );

                    tune_start_off =
                        off_counter.load(
                            std::memory_order_relaxed
                        );

                    tune_state_start =
                        now;

                    tune_state =
                        AutoTuneState::MEASURING;

                    std::cout
                        << "\nAuto Tune measurement "
                        << tune_iteration + 1
                        << "/"
                        << max_tune_iterations
                        << std::endl;
                }
            }

            // --------------------------------------------------------
            // MEASURING
            // --------------------------------------------------------
            else if (tune_state ==
                     AutoTuneState::MEASURING) {

                const double elapsed =
                    std::chrono::duration<double>(
                        now -
                        tune_state_start
                    ).count();

                char buffer[256];

                std::snprintf(
                    buffer,
                    sizeof(buffer),
                    "Auto Tune: measuring %.1f / %.1f sec",
                    elapsed,
                    measurement_seconds
                );

                tune_message =
                    buffer;

                if (elapsed >=
                    measurement_seconds) {

                    const uint64_t current_on =
                        on_counter.load(
                            std::memory_order_relaxed
                        );

                    const uint64_t current_off =
                        off_counter.load(
                            std::memory_order_relaxed
                        );

                    measured_on_rate =
                        static_cast<double>(
                            current_on -
                            tune_start_on
                        ) / elapsed;

                    measured_off_rate =
                        static_cast<double>(
                            current_off -
                            tune_start_off
                        ) / elapsed;

                    std::cout
                        << "\nMeasured background:"
                        << std::endl;

                    std::cout
                        << " ON  : "
                        << measured_on_rate / 1e6
                        << " Mev/s"
                        << std::endl;

                    std::cout
                        << " OFF : "
                        << measured_off_rate / 1e6
                        << " Mev/s"
                        << std::endl;

                    const bool on_ok =
                        measured_on_rate <=
                        target_on_rate;

                    const bool off_ok =
                        measured_off_rate <=
                        target_off_rate;

                    // =================================================
                    // Target reached
                    // =================================================
                    if (on_ok && off_ok) {

                        tune_state =
                            AutoTuneState::FINISHED;

                        tune_message =
                            "Auto Tune: FINISHED";

                        std::cout
                            << "\nAuto Tune finished."
                            << std::endl;

                        std::cout
                            << "bias_diff_on  = "
                            << biases.get(
                                "bias_diff_on"
                            )
                            << std::endl;

                        std::cout
                            << "bias_diff_off = "
                            << biases.get(
                                "bias_diff_off"
                            )
                            << std::endl;

                        try {

                            biases.save_to_file(
                                "silkyevcam_autotuned.bias"
                            );

                            std::cout
                                << "Saved: "
                                << "silkyevcam_autotuned.bias"
                                << std::endl;

                        } catch (
                            const std::exception &e
                        ) {

                            std::cerr
                                << "Could not save biases: "
                                << e.what()
                                << std::endl;
                        }

                    } else {

                        // =================================================
                        // Adjust biases
                        // =================================================

                        bool changed = false;

                        // ON noise too high
                        //
                        // Increase bias_diff_on
                        //
                        if (!on_ok) {

                            const int old_value =
                                biases.get(
                                    "bias_diff_on"
                                );

                            const int new_value =
                                old_value +
                                tune_step;

                            try {

                                if (biases.set(
                                    "bias_diff_on",
                                    new_value
                                )) {

                                    bias_diff_on_ctrl->value =
                                        biases.get(
                                            "bias_diff_on"
                                        );

                                    cv::setTrackbarPos(
                                        "bias_diff_on",
                                        window_name,
                                        bias_diff_on_ctrl->value
                                    );

                                    changed = true;

                                    std::cout
                                        << "bias_diff_on: "
                                        << old_value
                                        << " -> "
                                        << bias_diff_on_ctrl->value
                                        << std::endl;
                                }

                            } catch (
                                const std::exception &e
                            ) {

                                std::cerr
                                    << "bias_diff_on limit reached: "
                                    << e.what()
                                    << std::endl;
                            }
                        }

                        // OFF noise too high
                        //
                        // Decrease bias_diff_off
                        //
                        if (!off_ok) {

                            const int old_value =
                                biases.get(
                                    "bias_diff_off"
                                );

                            const int new_value =
                                old_value -
                                tune_step;

                            try {

                                if (biases.set(
                                    "bias_diff_off",
                                    new_value
                                )) {

                                    bias_diff_off_ctrl->value =
                                        biases.get(
                                            "bias_diff_off"
                                        );

                                    cv::setTrackbarPos(
                                        "bias_diff_off",
                                        window_name,
                                        bias_diff_off_ctrl->value
                                    );

                                    changed = true;

                                    std::cout
                                        << "bias_diff_off: "
                                        << old_value
                                        << " -> "
                                        << bias_diff_off_ctrl->value
                                        << std::endl;
                                }

                            } catch (
                                const std::exception &e
                            ) {

                                std::cerr
                                    << "bias_diff_off limit reached: "
                                    << e.what()
                                    << std::endl;
                            }
                        }

                        tune_iteration++;

                        // =================================================
                        // Stop if impossible
                        // =================================================
                        if (!changed ||
                            tune_iteration >=
                            max_tune_iterations) {

                            tune_state =
                                AutoTuneState::FAILED;

                            tune_message =
                                "Auto Tune: FAILED / limit reached";

                            std::cerr
                                << "\nAuto Tune stopped."
                                << std::endl;

                        } else {

                            // Allow sensor to settle after bias change
                            tune_state =
                                AutoTuneState::WAITING;

                            tune_state_start =
                                now;
                        }
                    }
                }
            }

            // ========================================================
            // Visualization
            // ========================================================
            cv::Mat display;

            {
                std::lock_guard<std::mutex>
                    lock(image_mutex);

                display =
                    event_image.clone();

                event_image.setTo(
                    cv::Scalar(
                        255,
                        255,
                        255
                    )
                );
            }

            // ========================================================
            // Information overlay
            // ========================================================
            char rate_text[256];

            std::snprintf(
                rate_text,
                sizeof(rate_text),
                "Total %.3f | ON %.3f | OFF %.3f Mev/s",
                event_rate / 1e6,
                on_rate / 1e6,
                off_rate / 1e6
            );

            cv::putText(
                display,
                rate_text,
                cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                0.60,
                cv::Scalar(0, 255, 0),
                2
            );

            // --------------------------------------------------------
            // Auto tune status
            // --------------------------------------------------------
            cv::putText(
                display,
                tune_message,
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX,
                0.60,
                cv::Scalar(0, 255, 255),
                2
            );

            // --------------------------------------------------------
            // Target
            // --------------------------------------------------------
            char target_text[256];

            std::snprintf(
                target_text,
                sizeof(target_text),
                "Target BG: ON < %.3f | OFF < %.3f Mev/s",
                target_on_rate / 1e6,
                target_off_rate / 1e6
            );

            cv::putText(
                display,
                target_text,
                cv::Point(10, 90),
                cv::FONT_HERSHEY_SIMPLEX,
                0.50,
                cv::Scalar(255, 255, 0),
                1
            );

            cv::imshow(
                window_name,
                display
            );

            if (using_fallback_reset_button) {
                cv::Mat controls_display(
                    68,
                    320,
                    CV_8UC3,
                    cv::Scalar(32, 32, 32)
                );

                const cv::Rect button_bounds(
                    10,
                    10,
                    300,
                    48
                );

                cv::rectangle(
                    controls_display,
                    button_bounds,
                    cv::Scalar(70, 120, 70),
                    cv::FILLED
                );

                cv::rectangle(
                    controls_display,
                    button_bounds,
                    cv::Scalar(150, 220, 150),
                    1
                );

                cv::putText(
                    controls_display,
                    "Restore startup biases",
                    cv::Point(31, 41),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.62,
                    cv::Scalar(255, 255, 255),
                    1,
                    cv::LINE_AA
                );

                cv::imshow(
                    controls_window_name,
                    controls_display
                );
            }

            // ========================================================
            // Keyboard
            // ========================================================
            const int key =
                cv::waitKey(10);

            // --------------------------------------------------------
            // Quit
            // --------------------------------------------------------
            if (key == 27 ||
                key == 'q' ||
                key == 'Q') {

                break;
            }

            // --------------------------------------------------------
            // Start auto tune
            // --------------------------------------------------------
            if (key == 'a' ||
                key == 'A') {

                if (tune_state ==
                        AutoTuneState::IDLE ||
                    tune_state ==
                        AutoTuneState::FINISHED ||
                    tune_state ==
                        AutoTuneState::FAILED) {

                    tune_iteration = 0;

                    tune_state =
                        AutoTuneState::WAITING;

                    tune_state_start =
                        std::chrono::steady_clock::now();

                    tune_message =
                        "Auto Tune: starting";

                    std::cout
                        << "\n================================"
                        << std::endl;

                    std::cout
                        << "AUTO TUNE START"
                        << std::endl;

                    std::cout
                        << "Keep camera and scene still."
                        << std::endl;

                    std::cout
                        << "Target ON : "
                        << target_on_rate / 1e6
                        << " Mev/s"
                        << std::endl;

                    std::cout
                        << "Target OFF: "
                        << target_off_rate / 1e6
                        << " Mev/s"
                        << std::endl;

                    std::cout
                        << "================================"
                        << std::endl;
                }
            }

            // --------------------------------------------------------
            // Restore startup biases
            // --------------------------------------------------------
            if (key == 'x' ||
                key == 'X') {

                restore_startup_biases();
            }

            // --------------------------------------------------------
            // Save biases manually
            // --------------------------------------------------------
            if (key == 's' ||
                key == 'S') {

                try {

                    biases.save_to_file(
                        "silkyevcam_custom.bias"
                    );

                    std::cout
                        << "Saved: "
                        << "silkyevcam_custom.bias"
                        << std::endl;

                } catch (
                    const std::exception &e
                ) {

                    std::cerr
                        << "Failed to save: "
                        << e.what()
                        << std::endl;
                }
            }

            // --------------------------------------------------------
            // Reload hardware values
            // --------------------------------------------------------
            if (key == 'r' ||
                key == 'R') {

                for (auto &ctrl : controls) {

                    try {

                        ctrl.value =
                            biases.get(
                                ctrl.name
                            );

                        cv::setTrackbarPos(
                            ctrl.name,
                            window_name,
                            ctrl.value
                        );

                    } catch (
                        const std::exception &e
                    ) {

                        std::cerr
                            << "Failed to read "
                            << ctrl.name
                            << ": "
                            << e.what()
                            << std::endl;
                    }
                }
            }
        }

        // ============================================================
        // Shutdown
        // ============================================================
        if (camera.is_running()) {
            camera.stop();
        }

        cv::destroyAllWindows();

        return 0;

    } catch (const std::exception &e) {

        std::cerr
            << "Fatal error: "
            << e.what()
            << std::endl;

        return 1;
    }
}

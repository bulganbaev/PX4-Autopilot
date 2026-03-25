#include <errno.h>
#include <unistd.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>

#include <board_config.h>

class MatekCamSwitch : public ModuleBase<MatekCamSwitch>, public ModuleParams
{
public:
	MatekCamSwitch() : ModuleParams(nullptr) {}
	~MatekCamSwitch() override = default;

	static int task_spawn(int argc, char *argv[])
	{
		_task_id = px4_task_spawn_cmd(
			"matek_cam_switch",
			SCHED_DEFAULT,
			SCHED_PRIORITY_DEFAULT,
			2200,
			(px4_main_t)&run_trampoline,
			(char *const *)argv);

		return _task_id < 0 ? -errno : 0;
	}

	static MatekCamSwitch *instantiate(int argc, char *argv[])
	{
		return new MatekCamSwitch();
	}

	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_DESCRIPTION(
			R"DESCR_STR(
Matek H743-Slim internal video GPIO mapper.

Uses PX4 logical AUX inputs from manual_control_setpoint:
- AUX1 -> GPIO_VIDEO_CAM (PD11) : camera select
- AUX2 -> GPIO_VIDEO_PWR (PD10) : Vsw on/off

Physical RC channels are configured with PX4 params:
- RC_MAP_AUX1 = channel for camera switch
- RC_MAP_AUX2 = channel for Vsw

Input range is [-1, 1].
Thresholds:
  >  0.5 = HIGH
  < -0.5 = LOW
Middle position keeps previous state.
)DESCR_STR");

		PRINT_MODULE_USAGE_NAME("matek_cam_switch", "driver");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_COMMAND("stop");
		PRINT_MODULE_USAGE_COMMAND("status");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}

	void run() override
	{
		PX4_INFO("started");

		bool last_cam_state = false;
		bool last_vsw_state = false;
		bool cam_initialized = false;
		bool vsw_initialized = false;

		while (!should_exit()) {
			manual_control_setpoint_s manual{};

			if (_manual_control_sub.update(&manual)) {

				// AUX1 -> camera select
				bool cam_state = last_cam_state;

				if (manual.aux1 > 0.5f) {
					cam_state = true;

				} else if (manual.aux1 < -0.5f) {
					cam_state = false;
				}

				if (!cam_initialized || cam_state != last_cam_state) {
					px4_arch_gpiowrite(GPIO_VIDEO_CAM, cam_state);
					last_cam_state = cam_state;
					cam_initialized = true;
					PX4_INFO("AUX1=%.2f -> VIDEO_CAM=%d", (double)manual.aux1, (int)cam_state);
				}

				// AUX2 -> Vsw
				bool vsw_state = last_vsw_state;

				if (manual.aux2 > 0.5f) {
					vsw_state = true;

				} else if (manual.aux2 < -0.5f) {
					vsw_state = false;
				}

				if (!vsw_initialized || vsw_state != last_vsw_state) {
					px4_arch_gpiowrite(GPIO_VIDEO_PWR, vsw_state);
					last_vsw_state = vsw_state;
					vsw_initialized = true;
					PX4_INFO("AUX2=%.2f -> VIDEO_PWR=%d", (double)manual.aux2, (int)vsw_state);
				}
			}

			usleep(20000); // 50 Hz
		}

		PX4_INFO("stopped");
	}
private:
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
};

extern "C" __EXPORT int matek_cam_switch_main(int argc, char *argv[])
{
	return MatekCamSwitch::main(argc, argv);
}

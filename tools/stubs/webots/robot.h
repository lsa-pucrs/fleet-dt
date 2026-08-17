/**
 * @file robot.h
 * @brief Syntax-check stub for the WeBots controller library.
 *
 * Declares only what adapters/webots/fdt_webots_controller.c calls. See
 * tools/stubs/mosquitto.h for why these exist.
 */
#ifndef FDT_STUB_WEBOTS_ROBOT_H
#define FDT_STUB_WEBOTS_ROBOT_H

void wb_robot_init(void);
int  wb_robot_step(int ms);
void wb_robot_cleanup(void);

#endif /* FDT_STUB_WEBOTS_ROBOT_H */

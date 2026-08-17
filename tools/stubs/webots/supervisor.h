/**
 * @file supervisor.h
 * @brief Syntax-check stub for the WeBots supervisor API.
 */
#ifndef FDT_STUB_WEBOTS_SUPERVISOR_H
#define FDT_STUB_WEBOTS_SUPERVISOR_H

typedef void *WbNodeRef;
typedef void *WbFieldRef;

WbNodeRef  wb_supervisor_node_get_from_def(const char *def);
WbFieldRef wb_supervisor_node_get_field(WbNodeRef node, const char *name);
const double *wb_supervisor_field_get_sf_vec3f(WbFieldRef field);
void wb_supervisor_field_set_sf_vec3f(WbFieldRef field, const double values[3]);
void wb_supervisor_field_set_sf_rotation(WbFieldRef field,
                                         const double values[4]);

#endif /* FDT_STUB_WEBOTS_SUPERVISOR_H */

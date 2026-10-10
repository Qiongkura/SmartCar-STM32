#ifndef LINE_MAP_H
#define LINE_MAP_H
#include <stdbool.h>
#include <stdint.h>

#define LM_SAMPLES 24
typedef struct {
  float x, y;
} lm_point_t;
typedef struct {
  /* Frame: X/Y fixed at launch; yaw is the car's clockwise-positive yaw. */
  float x, y, yaw, dist, prev_yaw;
  float axis_yaw, heading, offset;
  float incoming_heading, incoming_offset, corner_x, corner_y;
  float forward_cm, lateral_cm, pitch_cm;
  float integral, rot_cmd, last_observe_dist;
  lm_point_t points[LM_SAMPLES];
  int turns;
  uint8_t count, wr, confidence;
  bool track_valid, position_valid, incoming_valid, axis_valid, frozen;
} line_map_t;

void line_map_reset(line_map_t *m, float yaw, float dist,
                    float forward_cm, float lateral_cm, float pitch_cm);
void line_map_update(line_map_t *m, float dist, float yaw);
bool line_map_observe(line_map_t *m, uint8_t bits);
void line_map_begin_corner(line_map_t *m, int dir, float fallback_heading);
/* Constrained fit of the transverse arm. Two separated observations can
 * estimate position because its orientation is fixed perpendicular to the
 * measured incoming line; they are NOT a free two-line angle fit. */
bool line_map_corner_edges(line_map_t *m, int dir, uint8_t valid,
                          const float center_dist[8], const float yaw[8]);
float line_map_advance_remaining(const line_map_t *m);
bool line_map_observe_exit(line_map_t *m, uint8_t bits);
float line_map_cross_error(const line_map_t *m);
float line_map_steer(line_map_t *m, float yaw, float rate, float dt,
                     bool aligning);
bool line_map_aligned(const line_map_t *m, float yaw, float rate);
void line_map_finish_corner(line_map_t *m);
void line_map_lock_gap(line_map_t *m);
float line_map_angle_diff(float a, float b);
void line_map_sincos(float deg, float *s, float *c);
#endif

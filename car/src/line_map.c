#include "line_map.h"
#include <string.h>

static float absf(float x) { return x < 0 ? -x : x; }
static float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
float line_map_angle_diff(float a, float b) {
  float x = a - b;
  while (x > 180) x -= 360;
  while (x < -180) x += 360;
  return x;
}
/* Bhaskara approximation: max sine error about .0017; no new libm cost. */
static float sine(float deg) {
  deg = line_map_angle_diff(deg, 0);
  float sign = deg < 0 ? -1 : 1;
  deg = absf(deg);
  float z = deg * (180 - deg);
  return sign * (4 * z) / (40500 - z);
}
void line_map_sincos(float deg, float *s, float *c) { *s = sine(deg); *c = sine(deg + 90); }
static float atan_small(float x) {
  float z = x * x;
  return (x - x * z / 3 + x * z * z / 5) * 57.29578f;
}
static bool thin_line(uint8_t b, float *index) {
  int count = 0, sum = 0, first = -1, last = -1;
  for (int i = 0; i < 8; ++i) if (b & (1u << i)) {
    if (first < 0) first = i;
    last = i; ++count; sum += i;
  }
  if (!count || count > 3 || last - first + 1 != count) return false;
  *index = (float)sum / count - 3.5f;
  return true;
}
static lm_point_t sensor_point(const line_map_t *m, float index) {
  float s, c; line_map_sincos(m->yaw, &s, &c);
  float lateral = index * m->pitch_cm + m->lateral_cm;
  lm_point_t p = {m->x + m->forward_cm * c - lateral * s,
                  m->y + m->forward_cm * s + lateral * c};
  return p;
}
void line_map_reset(line_map_t *m, float yaw, float dist,
                    float forward_cm, float lateral_cm, float pitch_cm) {
  memset(m, 0, sizeof(*m));
  m->yaw = m->prev_yaw = m->axis_yaw = m->heading = yaw;
  m->dist = dist;
  m->forward_cm = forward_cm; m->lateral_cm = lateral_cm; m->pitch_cm = pitch_cm;
  m->last_observe_dist = dist - 1;
}
void line_map_update(line_map_t *m, float dist, float yaw) {
  float dd = dist - m->dist;
  if (absf(dd) <= 5 && absf(line_map_angle_diff(yaw, m->prev_yaw)) <= 30) {
    float s, c; line_map_sincos(m->prev_yaw + .5f * line_map_angle_diff(yaw, m->prev_yaw), &s, &c);
    m->x += dd * c; m->y += dd * s;
  }
  m->dist = dist; m->prev_yaw = m->yaw = yaw;
}
bool line_map_observe(line_map_t *m, uint8_t bits) {
  float index;
  if (m->frozen || !thin_line(bits, &index) ||
      absf(line_map_angle_diff(m->yaw, m->heading)) > 18 ||
      m->dist - m->last_observe_dist < .5f) return false;
  m->last_observe_dist = m->dist;
  m->points[m->wr] = sensor_point(m, index);
  m->wr = (m->wr + 1) % LM_SAMPLES;
  if (m->count < LM_SAMPLES) ++m->count;
  if (m->count < 8) return false;
  float s, c; line_map_sincos(m->heading, &s, &c);
  float sx=0, sy=0, sxx=0, sxy=0, lo=1e6f, hi=-1e6f;
  for (int i=0; i<m->count; ++i) {
    float x = c*m->points[i].x+s*m->points[i].y;
    float y = -s*m->points[i].x+c*m->points[i].y;
    sx+=x; sy+=y; sxx+=x*x; sxy+=x*y;
    if (x<lo) lo=x;
    if (x>hi) hi=x;
  }
  float n=m->count, den=n*sxx-sx*sx;
  if (hi-lo < 8 || den < 1) return false;
  float b=(n*sxy-sx*sy)/den, a=(sy-b*sx)/n;
  if (absf(b) > .27f) return false;
  float rss=0;
  for (int i=0; i<m->count; ++i) {
    float x=c*m->points[i].x+s*m->points[i].y;
    float y=-s*m->points[i].x+c*m->points[i].y;
    float e=y-a-b*x;
    if (absf(e)>1.0f) return false;
    rss+=e*e;
  }
  if (rss > n*.25f) return false;
  /* Update square axes from spatial line observations, not vehicle yaw EMA. */
  m->axis_yaw += clamp(.25f*atan_small(b), -.5f, .5f);
  m->heading=m->axis_yaw+90*m->turns;
  line_map_sincos(m->heading, &s, &c);
  float offset=0;
  for (int i=0; i<m->count; ++i) offset+=-s*m->points[i].x+c*m->points[i].y;
  m->offset=offset/n;
  m->track_valid=m->position_valid=m->axis_valid=true; m->confidence=2;
  return true;
}
void line_map_begin_corner(line_map_t *m, int dir, float fallback_heading) {
  if (!m->axis_valid) {
    m->heading=fallback_heading;
    m->axis_yaw=fallback_heading-90*m->turns;
    float s,c; line_map_sincos(m->heading,&s,&c);
    m->offset=-s*m->x+c*m->y;
    if (m->count) {
      m->offset=0;
      for(int i=0;i<m->count;++i) m->offset+=-s*m->points[i].x+c*m->points[i].y;
      m->offset/=m->count;
    }
  }
  m->incoming_valid=m->position_valid || m->count>=3;
  m->incoming_heading=m->heading; m->incoming_offset=m->offset;
  float s,c; line_map_sincos(m->incoming_heading,&s,&c);
  lm_point_t p=sensor_point(m,0);
  float along=p.x*c+p.y*s;
  m->corner_x=along*c-m->incoming_offset*s;
  m->corner_y=along*s+m->incoming_offset*c;
  m->turns += dir;
  m->heading=m->axis_yaw+90*m->turns;
  line_map_sincos(m->heading,&s,&c);
  m->offset=-s*m->corner_x+c*m->corner_y;
  m->position_valid=false; m->confidence=1; m->frozen=true;
  m->integral=m->rot_cmd=0;
}
bool line_map_corner_edges(line_map_t *m, int dir, uint8_t valid,
                          const float center_dist[8], const float yaw[8]) {
  float s,c; line_map_sincos(m->incoming_heading,&s,&c);
  float along[8], sum=0; int n=0, first=-1, last=-1;
  for (int i=0;i<8;++i) {
    /* Centre probes can be observing the incoming longitudinal arm. */
    if (!(valid&(1u<<i)) || (dir<0 ? i>2 : i<5)) continue;
    if (m->dist-center_dist[i]<-.5f || m->dist-center_dist[i]>10 ||
        absf(line_map_angle_diff(yaw[i],m->incoming_heading))>8) continue;
    float es,ec; line_map_sincos(yaw[i],&es,&ec);
    float lateral=((float)i-3.5f)*m->pitch_cm+m->lateral_cm;
    float forward=center_dist[i]-m->dist+m->forward_cm;
    float x=m->x+forward*ec-lateral*es;
    float y=m->y+forward*es+lateral*ec;
    along[n]=x*c+y*s; sum+=along[n++];
    if(first<0)first=i;
    last=i;
  }
  if (n<2 || (last-first)*m->pitch_cm<1.0f) return false;
  float mean=sum/n;
  for(int i=0;i<n;++i) if(absf(along[i]-mean)>.45f) return false;
  float old=m->corner_x*c+m->corner_y*s;
  if(absf(mean-old)>4) return false;
  m->corner_x=mean*c-m->incoming_offset*s;
  m->corner_y=mean*s+m->incoming_offset*c;
  line_map_sincos(m->heading,&s,&c);
  m->offset=-s*m->corner_x+c*m->corner_y;
  m->position_valid=m->incoming_valid; m->confidence=m->position_valid?3:1;
  return true;
}
float line_map_advance_remaining(const line_map_t *m) {
  float s,c; line_map_sincos(m->incoming_heading,&s,&c);
  return (m->corner_x-m->x)*c+(m->corner_y-m->y)*s;
}
bool line_map_observe_exit(line_map_t *m, uint8_t bits) {
  float index;
  if (!thin_line(bits,&index) || absf(line_map_angle_diff(m->yaw,m->heading))>12) return false;
  lm_point_t p=sensor_point(m,index);
  float s,c; line_map_sincos(m->heading,&s,&c);
  float observed=-s*p.x+c*p.y;
  if (m->position_valid && absf(observed-m->offset)>2.5f) return false;
  if (m->position_valid) m->offset+=.15f*(observed-m->offset);
  else m->offset=observed;
  m->position_valid=true; m->confidence=3;
  return true;
}
float line_map_cross_error(const line_map_t *m) {
  float s,c; line_map_sincos(m->heading,&s,&c);
  return m->offset-(-s*m->x+c*m->y);
}
float line_map_steer(line_map_t *m, float yaw, float rate, float dt, bool aligning) {
  if (dt<=0 || dt>.05f) dt=.01f;
  float correction=0;
  if(m->position_valid) correction=clamp(4*line_map_cross_error(m),aligning?-10:-3,aligning?10:3);
  float err=line_map_angle_diff(m->heading+correction,yaw);
  m->integral=clamp(m->integral+err*dt,-10,10);
  float rot=clamp(4.5f*err+.5f*m->integral-1.2f*rate,-35,35);
  m->rot_cmd+=clamp(rot-m->rot_cmd,-180*dt,180*dt);
  return m->rot_cmd;
}
bool line_map_aligned(const line_map_t *m,float yaw,float rate) {
  return absf(line_map_angle_diff(m->heading,yaw))<1.5f && absf(rate)<8 &&
         (!m->position_valid || absf(line_map_cross_error(m))<.8f);
}
void line_map_finish_corner(line_map_t *m) {
  m->frozen=false; m->track_valid=false; m->count=m->wr=0;
  m->last_observe_dist=m->dist; m->integral=0;
}
void line_map_lock_gap(line_map_t *m) { m->frozen=true; m->integral=0; }

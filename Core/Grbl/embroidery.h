#ifndef EMBROIDERY_H
#define EMBROIDERY_H

/* Pedal PWM is expressed in permille, independent of GRBL $30/$31. */
#define DEFAULT_EMBROIDERY_NEUTRAL_PWM 500U
#define DEFAULT_EMBROIDERY_TRIM_PWM 200U
#define DEFAULT_EMBROIDERY_LONG_PWM 740U
#define DEFAULT_EMBROIDERY_SHORT_PWM 860U
#define EMBROIDERY_TRIM_MS 1000U
#ifndef EMBROIDERY_AUTO_SPEED
#define EMBROIDERY_AUTO_SPEED 0
#endif

/* Interpolate against squared XY distance: no square root is needed. */
static inline unsigned short embroidery_speed_pwm(float distance_squared,
                                                  unsigned short short_pwm,
                                                  unsigned short long_pwm)
{
  if (distance_squared <= 4.0f) { return short_pwm; }
  if (distance_squared >= 64.0f) { return long_pwm; }
  return (unsigned short)((float)short_pwm - ((float)short_pwm - long_pwm) *
                         (distance_squared - 4.0f) / 60.0f);
}

#endif

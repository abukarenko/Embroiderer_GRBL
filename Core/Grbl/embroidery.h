#ifndef EMBROIDERY_H
#define EMBROIDERY_H

/* Pedal PWM is expressed in permille, independent of GRBL $30/$31. */
#define EMBROIDERY_NEUTRAL_PWM 500U
#define EMBROIDERY_TRIM_PWM 400U
#define EMBROIDERY_TRIM_MS 1000U
#ifndef EMBROIDERY_AUTO_SPEED
#define EMBROIDERY_AUTO_SPEED 0
#endif

/* Interpolate against squared XY distance: no square root is needed. */
static inline unsigned short embroidery_speed_pwm(float distance_squared)
{
  if (distance_squared <= 4.0f) { return 860U; }
  if (distance_squared >= 144.0f) { return 650U; }
  return (unsigned short)(860.0f - 210.0f *
                         (distance_squared - 4.0f) / 140.0f);
}

#endif

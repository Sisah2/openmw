// Narkowicz 2015, "ACES Filmic Tone Mapping Curve"
vec3 aces(vec3 x) {
  const float a = 2.51;
  const float b = 0.03;
  const float c = 2.43;
  const float d = 0.59;
  const float e = 0.14;
  return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

float aces(float x) {
  const float a = 2.51;
  const float b = 0.03;
  const float c = 2.43;
  const float d = 0.59;
  const float e = 0.14;
  return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

vec3 preLight(vec3 x)
{
    return pow(x, vec3(2.2));
}

vec3 toneMap(vec3 x)
{
#ifdef PER_CHANEL
    vec3 col = x;//= pow(x, vec3(2.2));
    col.x = aces(col.x);
    col.y = aces(col.y);
    col.z = aces(col.z);
#else
    vec3 col = aces(x);
#endif

    return  pow(col, vec3( 1.0 / 2.2 ));
}


// Unreal 3, Documentation: "Color Grading"
// Adapted to be close to Tonemap_ACES, with similar range
// Gamma 2.2 correction is baked in, don't use with sRGB conversion!
vec3 unreal(vec3 x) {
  return x / (x + 0.155) * 1.019;
}

float unreal(float x) {
  return x / (x + 0.155) * 1.019;
}

vec3 preLight(vec3 x)
{
    return pow(x, vec3(2.2));
}

vec3 toneMap(vec3 x)
{
#ifdef PER_CHANEL
    vec3 col = x;//= pow(x, vec3(2.2));
    col.x = unreal(col.x);
    col.y = unreal(col.y);
    col.z = unreal(col.z);
#else
    vec3 col = unreal(x);
#endif

    return  col;
}


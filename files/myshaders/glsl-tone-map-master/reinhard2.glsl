vec3 reinhard2(vec3 x) {
  const float L_white = 4.0;

  return (x * (1.0 + x / (L_white * L_white))) / (1.0 + x);
}

float reinhard2(float x) {
  const float L_white = 4.0;

  return (x * (1.0 + x / (L_white * L_white))) / (1.0 + x);
}

vec3 preLight(vec3 x)
{
    return pow(x, vec3(2.2));
}

vec3 toneMap(vec3 x)
{
#ifdef PER_CHANEL
    vec3 col = x;//= pow(x, vec3(2.2));
    col.x = reinhard2(col.x);
    col.y = reinhard2(col.y);
    col.z = reinhard2(col.z);
#else
    vec3 col = reinhard2(x);
#endif

    return  pow(col, vec3( 1.0 / 2.2 ));
}


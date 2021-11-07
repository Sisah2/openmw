vec3 reinhard(vec3 x) {
  return x / (1.0 + x);
}

float reinhard(float x) {
  return x / (1.0 + x);
}

vec3 preLight(vec3 x)
{
    return pow(x, vec3(2.2));
}

vec3 toneMap(vec3 x)
{
#ifdef PER_CHANEL
    vec3 col = x;//= pow(x, vec3(2.2));
    col.x = reinhard(col.x);
    col.y = reinhard(col.y);
    col.z = reinhard(col.z);
#else
    vec3 col = reinhard(x);
#endif

    return  pow(col, vec3( 1.0 / 2.2 ));
}



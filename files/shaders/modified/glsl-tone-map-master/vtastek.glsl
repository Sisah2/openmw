
vec3 toneMap(vec3 color)
{
	float A = 0.105;
	float B = 0.714;
	float C = 0.57;
	float D = 0.21;
	float E = 0.092;
	float F = 0.886;
	float W = 600.0;
	float exposure = 2.;
	color *= exposure * 2.0;
	color = ((color * (A * color + C * B) + D * E) / (color * (A * color + B) + D * F)) - E / F;
	float white = ((W * (A * W + C * B) + D * E) / (W * (A * W + B) + D * F)) - E / F;
	color /= white;
	color = pow(color, vec3( 1.0 / 2.2 ));
  return color;
}

vec3 preLight(vec3 x)
{
    return pow(x, vec3(2.2));
}

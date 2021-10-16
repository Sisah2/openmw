uniform int tonemaper;

#define NONE 0
#define ACES 1
#define FILMIC 2
#define LOTTES 3
#define REINHARD 4
#define REINHARD2 5
#define UCHIMURA 6
#define UNCHARTED2 7
#define UNREAL 8
#define VTASTEK 9

vec3 aces(vec3 x) {
  const float a = 2.51;
  const float b = 0.03;
  const float c = 2.43;
  const float d = 0.59;
  const float e = 0.14;
  return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

vec3 tonemapFilmic(vec3 x) {
  vec3 X = max(vec3(0.0), x - 0.004);
  vec3 result = (X * (6.2 * X + 0.5)) / (X * (6.2 * X + 1.7) + 0.06);
  return pow(result, vec3(2.2));
}

vec3 lottes(vec3 x) {
  const vec3 a = vec3(1.6);
  const vec3 d = vec3(0.977);
  const vec3 hdrMax = vec3(8.0);
  const vec3 midIn = vec3(0.18);
  const vec3 midOut = vec3(0.267);

  const vec3 b =
      (-pow(midIn, a) + pow(hdrMax, a) * midOut) /
      ((pow(hdrMax, a * d) - pow(midIn, a * d)) * midOut);
  const vec3 c =
      (pow(hdrMax, a * d) * pow(midIn, a) - pow(hdrMax, a) * pow(midIn, a * d) * midOut) /
      ((pow(hdrMax, a * d) - pow(midIn, a * d)) * midOut);

  return pow(x, a) / (pow(x, a * d) * b + c);
}

vec3 reinhard(vec3 x) {
  return x / (1.0 + x);
}

vec3 reinhard2(vec3 x) {
  const float L_white = 4.0;

  return (x * (1.0 + x / (L_white * L_white))) / (1.0 + x);
}

vec3 uchimura(vec3 x, float P, float a, float m, float l, float c, float b) {
  float l0 = ((P - m) * l) / a;
  float L0 = m - m / a;
  float L1 = m + (1.0 - m) / a;
  float S0 = m + l0;
  float S1 = m + a * l0;
  float C2 = (a * P) / (P - S1);
  float CP = -C2 / P;

  vec3 w0 = vec3(1.0 - smoothstep(0.0, m, x));
  vec3 w2 = vec3(step(m + l0, x));
  vec3 w1 = vec3(1.0 - w0 - w2);

  vec3 T = vec3(m * pow(x / m, vec3(c)) + b);
  vec3 S = vec3(P - (P - S1) * exp(CP * (x - S0)));
  vec3 L = vec3(m + a * (x - m));

  return T * w0 + L * w1 + S * w2;
}

vec3 uchimura(vec3 x) {
  const float P = 1.0;  // max display brightness
  const float a = 1.0;  // contrast
  const float m = 0.22; // linear section start
  const float l = 0.4;  // linear section length
  const float c = 1.33; // black
  const float b = 0.0;  // pedestal

  return uchimura(x, P, a, m, l, c, b);
}

vec3 uncharted2Tonemap(vec3 x) {
  float A = 0.15;
  float B = 0.50;
  float C = 0.10;
  float D = 0.20;
  float E = 0.02;
  float F = 0.30;
  float W = 11.2;
  return ((x * (A * x + C * B) + D * E) / (x * (A * x + B) + D * F)) - E / F;
}

vec3 uncharted2(vec3 color) {
  const float W = 11.2;
  float exposureBias = 2.0;
  vec3 curr = uncharted2Tonemap(exposureBias * color);
  vec3 whiteScale = 1.0 / uncharted2Tonemap(vec3(W));
  return curr * whiteScale;
}

vec3 unreal(vec3 x) {
  return x / (x + 0.155) * 1.019;
}

vec3 vtastek(vec3 color)
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
  return color;
}

vec3 preLight(vec3 x)
{
    if(tonemaper == ACES || tonemaper == FILMIC || tonemaper == LOTTES || tonemaper == REINHARD || tonemaper == REINHARD2 || tonemaper == UCHIMURA || tonemaper == UNCHARTED2 || tonemaper == UNREAL || tonemaper == VTASTEK)
        return pow(x, vec3(2.2));
    else
        return x;
}

vec3 toneMap(vec3 x)
{
    if(tonemaper == NONE) return x;

    else if(tonemaper == ACES) x = aces(x);
    else if(tonemaper == FILMIC) x = tonemapFilmic(x);
    else if(tonemaper == LOTTES) x = lottes(x);
    else if(tonemaper == REINHARD) x = reinhard(x);
    else if(tonemaper == REINHARD2) x = reinhard2(x);
    else if(tonemaper == UCHIMURA) x = uchimura(x);
    else if(tonemaper == UNCHARTED2) x = uncharted2(x);
    else if(tonemaper == UNREAL) return unreal(x);
    else if(tonemaper == VTASTEK) return vtastek(x);

    return  pow(x, vec3( 1.0 / 2.2 ));
}
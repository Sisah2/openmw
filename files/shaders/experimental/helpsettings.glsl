#define LINEAR 1

//#define HEIGHT_FOG
//#define DYNAMICHFOG
//#define ANIMATED_HEIGHT_FOG
const float fogheight = 5.0;
const float foghdistance = 1.0;
const float maxfheight = 1.0;

// underwater fog setting, need to find better config vec3(start, end, contrib)
const vec3 uwdeepfog = vec3(-1000.0, 1500.0, 1.0); //deeper terrain/objects become more fogged
const vec3 uwdistfog = vec3(-3333.0, 6666.0, 0.15); //distant underwater terrain/objects become more fogged
const vec3 uwfogcolor = vec3(12.0/255.0, 30.0/255.0, 37.0/255.0);
//#define UNDERWATER_DISTORTION

// fade objects normal, specular and env maps at start distance, skip them at end distance
//#define NORMAL_MAP_FADING
const vec2 nmfader = vec2(7455.0, 8196.0);

// some extra grass displacement during storms
#define STORM_MODE


uniform int tonemaper;

//const int tonemaper = 0;

//const bool linearLighting = false;
//const bool Aces = false;
//const bool Filmic = false;



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

vec3 SRGBToLinearApprox(vec3 sRGB) {
	vec3 RGB = sRGB * (sRGB * (sRGB * 0.305306011 + 0.682171111) + 0.012522878);
	return RGB;
}

vec3 LessThan(vec3 f, float value) {
	return vec3(
		(f.x < value) ? 1.0 : 0.0,
		(f.y < value) ? 1.0 : 0.0,
		(f.z < value) ? 1.0 : 0.0);
}

vec3 LinearToSRGB(vec3 rgb) {
	rgb = clamp(rgb, 0.0, 1.0);

	return mix(
		pow(rgb, vec3(1.0 / 2.4)) * 1.055 - 0.055,
		rgb * 12.92,
		LessThan(rgb, 0.0031308)
	);
}

vec3 SRGBToLinear(vec3 rgb) {
	rgb = clamp(rgb, 0.0, 1.0);

	return mix(
		pow(((rgb + 0.055) / 1.055), vec3(2.4)),
		rgb / 12.92,
		LessThan(rgb, 0.04045)
	);
}

vec3 texLoad(vec3 x)
{
#if @linearLighting
    return SRGBToLinear(x);
#else
    return x;
#endif
}

vec3 colLoad(vec3 x)
{
#if @linearLighting
    return SRGBToLinearApprox(x);
#else
    return x;
#endif
}

vec3 vcolLoad(vec3 x) {
#if @linearLighting
    return sqrt(x);
#else
    return x;
#endif
}

#define PI 3.141592653589793238462643383279502884197169
float Fd_Lambert() {
#if @linearLighting
    return 1.0 / PI;
#else
    return 1.0;
#endif
}

float getExposure(float x) {
	return mix(3.14, 3.14, x);
}

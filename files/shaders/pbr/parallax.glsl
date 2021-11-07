#define PARALLAX_SCALE 0.04
#define PARALLAX_BIAS -0.02

#if (@normalMap && @parallax)
void getParallaxOffset(inout vec2 adjustedUV, vec3 eyeDir, mat3 tbnTranspose, sampler2D normalmap, float flipY)
{
    vec3 TSeyeDir = eyeDir;
	
	for(int i = 0; i < 4; i++) {
	vec4 Normal = texture2D(normalMap, adjustedUV.xy);
	float h = Normal.a;
	adjustedUV += (h - 0.6) * 0.015 * Normal.z * TSeyeDir.xy;
	}
}
#endif

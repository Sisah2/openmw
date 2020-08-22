#if @parallax
#define PARALLAX_SCALE 0.04
#define PARALLAX_BIAS -0.02

vec2 getParallaxOffset(vec3 eyeDir, mat3 tbnTranspose, float height, float flipY)
{
    vec3 TSeyeDir = normalize(eyeDir * tbnTranspose);
    return vec2(TSeyeDir.x, TSeyeDir.y * flipY) * ( height * PARALLAX_SCALE + PARALLAX_BIAS );
}
#endif

vec3 getSpecular(vec3 viewNormal, vec3 viewDirection, float shininess, vec3 matSpec)
{
    vec3 lightDir = normalize(gl_LightSource[0].position.xyz);
    float NdotL = dot(viewNormal, lightDir);
    if (NdotL <= 0.0)
        return vec3(0.,0.,0.);
    vec3 halfVec = normalize(lightDir - viewDirection);
    float NdotH = dot(viewNormal, halfVec);
    return pow(max(NdotH, 0.0), max(1e-4, shininess)) * gl_LightSource[0].specular.xyz * matSpec;
}

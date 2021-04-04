#define LIGHTING_MODEL_FFP 0
#define LIGHTING_MODEL_SINGLE_UBO 1
#define LIGHTING_MODEL_PER_OBJECT_UNIFORM 2

#if !@ffpLighting
#define getLight LightBuffer

float quickstep(float x)
{
    x = clamp(x, 0.0, 1.0);
    x = 1.0 - x*x;
    x = 1.0 - x*x;
    return x;
}

struct LightData
{
    vec4 position;
    vec4 diffuse;
    vec4 ambient;
    vec4 specular;
    vec4 attenuation;   // constant, linear, quadratic, radius
};

uniform LightData LightBuffer[@maxLights];
uniform int PointLightCount;

#else
#define getLight gl_LightSource
#endif

void perLightSun(out vec3 ambientOut, out vec3 diffuseOut, vec3 viewPos, vec3 viewNormal)
{    
    vec3 lightDir = normalize(getLight[0].position.xyz);

    ambientOut = getLight[0].ambient.xyz;
    vec3 sunDiffuse = getLight[0].diffuse.xyz;
    float lambert = dot(viewNormal.xyz, lightDir);
#ifndef GRASS
    lambert = max(lambert, 0.0);
#else
    float eyeCosine = dot(normalize(viewPos), viewNormal.xyz);
    if (lambert < 0.0)
    {
        lambert = -lambert;
        eyeCosine = -eyeCosine;
    }
    lambert *= clamp(-8.0 * (1.0 - 0.3) * eyeCosine + 1.0, 0.3, 1.0);
#endif

    diffuseOut = sunDiffuse * lambert;
}

void perLightPoint(out vec3 ambientOut, out vec3 diffuseOut, int lightIndex, vec3 viewPos, vec3 viewNormal)
{
    vec3 lightDir = getLight[lightIndex].position.xyz - viewPos;

    float lightDistance = length(lightDir);

#if !@ffpLighting
    // This has a *considerable* performance uplift where GPU is a bottleneck
    if (lightDistance > getLight[lightIndex].attenuation.w * 2.0)
    {
        ambientOut = vec3(0.0);
        diffuseOut = vec3(0.0);
        return;
    }
#endif

    lightDir = normalize(lightDir);

#if @ffpLighting
    float illumination = clamp(1.0 / (getLight[lightIndex].constantAttenuation + getLight[lightIndex].linearAttenuation * lightDistance + getLight[lightIndex].quadraticAttenuation * lightDistance * lightDistance), 0.0, 1.0);
#else
    float illumination = clamp(1.0 / (getLight[lightIndex].attenuation.x + getLight[lightIndex].attenuation.y * lightDistance + getLight[lightIndex].attenuation.z * lightDistance * lightDistance), 0.0, 1.0);
    illumination *= 1.0 - quickstep((lightDistance / (getLight[lightIndex].attenuation.w)) - 1.0);
#endif

    ambientOut = getLight[lightIndex].ambient.xyz * illumination;
    float lambert = dot(viewNormal.xyz, lightDir) * illumination;
    
#ifndef GRASS
    lambert = max(lambert, 0.0);
#else
    float eyeCosine = dot(normalize(viewPos), viewNormal.xyz);
    if (lambert < 0.0)
    {
        lambert = -lambert;
        eyeCosine = -eyeCosine;
    }
    lambert *= clamp(-8.0 * (1.0 - 0.3) * eyeCosine + 1.0, 0.3, 1.0);
#endif

    diffuseOut = getLight[lightIndex].diffuse.xyz * lambert;
}

#if PER_PIXEL_LIGHTING
void doLighting(vec3 viewPos, vec3 viewNormal, float shadowing, out vec3 diffuseLight, out vec3 ambientLight)
#else
void doLighting(vec3 viewPos, vec3 viewNormal, out vec3 diffuseLight, out vec3 ambientLight, out vec3 shadowDiffuse)
#endif
{
    vec3 ambientOut, diffuseOut;
    // This light gets added a second time in the loop to fix Mesa users' slowdown, so we need to negate its contribution here.
    perLightSun(ambientOut, diffuseOut, viewPos, viewNormal);

#if PER_PIXEL_LIGHTING
    diffuseLight = diffuseOut * shadowing - diffuseOut;
#endif
    ambientLight = gl_LightModel.ambient.xyz;

    perLightSun(ambientOut, diffuseOut, viewPos, viewNormal);
    ambientLight += ambientOut;
    diffuseLight += diffuseOut;

#if @lightingModel == LIGHTING_MODEL_FFP
    for (int i=1; i < @maxLights; ++i)
    {
#else
    for (int i=1; i <= PointLightCount; ++i)
    {
#endif
        perLightPoint(ambientOut, diffuseOut, i, viewPos, viewNormal);
        ambientLight += ambientOut;
        diffuseLight += diffuseOut;
    }
}

vec3 getSpecular(vec3 viewNormal, vec3 viewDirection, float shininess, vec3 matSpec)
{
    vec3 sunDir = getLight[0].position.xyz;
    vec3 sunSpec = getLight[0].specular.xyz;

    vec3 lightDir = normalize(sunDir);
    float NdotL = dot(viewNormal, lightDir);
    if (NdotL <= 0.0)
        return vec3(0.0);
    vec3 halfVec = normalize(lightDir - viewDirection);
    float NdotH = dot(viewNormal, halfVec);
    return pow(max(NdotH, 0.0), max(1e-4, shininess)) * sunSpec * matSpec;
}


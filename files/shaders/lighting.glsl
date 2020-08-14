#define MAX_LIGHTS 8

const int ColorMode_None = 0;
const int ColorMode_Emission = 1;
const int ColorMode_AmbientAndDiffuse = 2;
const int ColorMode_Ambient = 3;
const int ColorMode_Diffuse = 4;
const int ColorMode_Specular = 5;

void perLight(out vec3 ambientOut, out vec3 diffuseOut, int lightIndex, vec3 viewPos, vec3 viewNormal, vec4 diffuse, vec3 ambient, bool isGrass)
{
    vec3 lightDir;
    float lightDistance;

    lightDir = gl_LightSource[lightIndex].position.xyz - (viewPos.xyz * gl_LightSource[lightIndex].position.w);
    lightDistance = length(lightDir);
    lightDir = normalize(lightDir);
    float illumination = clamp(1.0 / (gl_LightSource[lightIndex].constantAttenuation + gl_LightSource[lightIndex].linearAttenuation * lightDistance + gl_LightSource[lightIndex].quadraticAttenuation * lightDistance * lightDistance), 0.0, 1.0);

    ambientOut = ambient * gl_LightSource[lightIndex].ambient.xyz * illumination;

#if (@particleHandling == 2 || @particleHandling == 4)
    diffuseOut = diffuse.xyz * gl_LightSource[lightIndex].diffuse.xyz * 0.5196 * illumination;
#else
    if (isGrass)
        diffuseOut = diffuse.xyz * gl_LightSource[lightIndex].diffuse.xyz * (max(dot(viewNormal.xyz, lightDir), 0.0) + max(dot(-viewNormal.xyz, lightDir), 0.0)) * illumination;
    else
        diffuseOut = diffuse.xyz * gl_LightSource[lightIndex].diffuse.xyz * max(dot(viewNormal.xyz, lightDir), 0.0) * illumination;
#endif
}

vec4 doLighting(vec3 viewPos, vec3 viewNormal, vec4 vertexColor, bool isGrass)
{
    vec4 diffuse;
    vec3 ambient;
    if (colorMode == ColorMode_AmbientAndDiffuse)
    {
        diffuse = vertexColor;
        ambient = vertexColor.xyz;
    }
    else if (colorMode == ColorMode_Diffuse)
    {
        diffuse = vertexColor;
        ambient = gl_FrontMaterial.ambient.xyz;
    }
    else if (colorMode == ColorMode_Ambient)
    {
        diffuse = gl_FrontMaterial.diffuse;
        ambient = vertexColor.xyz;
    }
    else
    {
        diffuse = gl_FrontMaterial.diffuse;
        ambient = gl_FrontMaterial.ambient.xyz;
    }
    vec4 lightResult = vec4(0.0, 0.0, 0.0, diffuse.a);

    vec3 diffuseLight, ambientLight;
    perLight(ambientLight, diffuseLight, 0, viewPos, viewNormal, diffuse, ambient, isGrass);

    for (int i=0; i<MAX_LIGHTS; ++i)
    {
        perLight(ambientLight, diffuseLight, i, viewPos, viewNormal, diffuse, ambient, isGrass);
        lightResult.xyz += ambientLight + diffuseLight;
    }

    lightResult.xyz += gl_LightModel.ambient.xyz * ambient;

    if (colorMode == ColorMode_Emission)
        lightResult.xyz += vertexColor.xyz;
    else
        lightResult.xyz += gl_FrontMaterial.emission.xyz;

#if @clamp
    lightResult = clamp(lightResult, vec4(0.0), vec4(1.0));
#else
    lightResult = max(lightResult, 0.0);
#endif
    return lightResult;
}
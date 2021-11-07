#include "lightsettings.glsl"

vec3 ToLinearColApprox(vec3 col) {
    return col * col;
}

vec3 Remap(vec3 original_value, vec3 new_min, vec3 new_max)
{
   return new_min + (original_value * (new_max - new_min));
}

float orenNayarDiffuse(
  vec3 lightDirection,
  vec3 viewDirection,
  vec3 surfaceNormal,
  float roughness) {
  
  float LdotV = dot(lightDirection, viewDirection);
  float NdotL = dot(lightDirection, surfaceNormal);
  float NdotV = dot(surfaceNormal, viewDirection);

  float s = LdotV - NdotL * NdotV;
  float t = mix(1.0, max(NdotL, NdotV), step(0.0, s));

  float sigma2 = roughness * roughness;
  float A = 1.0 + sigma2 * (0.18 / (sigma2 + 0.13) + 0.5 / (sigma2 + 0.33));
  float B = 0.45 * sigma2 / (sigma2 + 0.09);

  return max(0.0, NdotL) * (A + B * s / t) / 3.1415;
}

void LightSun(out vec3 ambientOut, out vec3 diffuseOut, int lightIndex, vec3 viewPos, vec3 viewNormal, vec4 diffuse, vec3 ambient)
{
    vec3 lightDir = normalize(lcalcPosition(0).xyz);
    vec3 sunbounce = mix(vec3(0.0), lcalcDiffuse(lightIndex), gl_LightModel.ambient.xyz);

    float lambert = dot(viewNormal.xyz, -lightDir);

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

	  sunbounce = sunbexp * mix(sunbounce, vec3(0.18), 0.5 * lcalcDiffuse(lightIndex).x) * clamp((lambert + 0.4) /((1.0 + 0.4) * (1.0 + 0.4)), 0.0 ,1.0);
		
    ambientOut = sunbounce;    
    diffuseOut = sunexp * diffuse.xyz * ToLinearColApprox(lcalcDiffuse(lightIndex)) * orenNayarDiffuse(lightDir, viewNormal.xyz,viewNormal.xyz, 0.1);
}

void perLight(out vec3 ambientOut, out vec3 diffuseOut, int lightIndex, vec3 viewPos, vec3 viewNormal, vec4 diffuse, vec3 ambient)
{
    vec3 lightDir = lcalcPosition(lightIndex) - viewPos.xyz;
    float lightDistance = length(lightDir);
    lightDir = normalize(lightDir);

// cull non-FFP point lighting by radius, light is guaranteed to not fall outside this bound with our cutoff
#if !@lightingMethodFFP
    if (lightDistance > lcalcRadius(lightIndex) * 2.0)
    {
        ambientOut = vec3(0.0);
        diffuseOut = vec3(0.0);
        return;
    }
#endif

    float illumination = lcalcIllumination(lightIndex, lightDistance);

    float lambert = dot(viewNormal.xyz, lightDir) * illumination;
#ifndef GRASS
    lambert = clamp(lambert, 0.0, 1.0);
#else
    float eyeCosine = dot(normalize(viewPos), viewNormal.xyz);
    if (lambert < 0.0)
    {
        lambert = -lambert;
        eyeCosine = -eyeCosine;
    }
    lambert *= clamp(-8.0 * (1.0 - 0.3) * eyeCosine + 1.0, 0.3, 1.0);
#endif

    ambientOut = vec3(0.0);
    diffuseOut = mix(pnightexp, pdayexp, lcalcDiffuse(0).x) * diffuse.xyz * ToLinearColApprox(lcalcDiffuse(lightIndex)) * lambert;
}

#if PER_PIXEL_LIGHTING
vec3 doLighting(vec3 viewPos, vec3 viewNormal, vec4 vertexColor, float shadowing)
#else
vec3 doLighting(vec3 viewPos, vec3 viewNormal, vec4 vertexColor)
#endif
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
    vec3 ambientLight, diffuseLight;
    LightSun(ambientLight, diffuseLight, 0, viewPos, viewNormal, diffuse, ambient);

    float interiorb = 0.0;
#ifdef OBJECT
    if(isInterior)
    {
        interiorb = 1.0;
        ambientLight *= vec3(intsunlight);
        diffuseLight *= vec3(intsunlight);
    }
#endif

    lightResult.xyz += (diffuseLight + ambientLight);

    #if PER_PIXEL_LIGHTING
        lightResult.xyz += diffuseLight * shadowing - diffuseLight; // This light gets added a second time in the loop to fix Mesa users' slowdown, so we need to negate its contribution here.
    #endif

    for (int i = @startLight; i < @endLight; ++i)
    {
        diffuseLight = vec3(0.0);

        perLight(ambientLight, diffuseLight, i, viewPos, viewNormal, diffuse, ambient);

#ifdef OBJECT
    if(lcalcDiffuse(i).x < 0.0 && isInterior)
        diffuseLight *= vec3(-1.0);
#endif

       if(diffuseLight != vec3(0.0))
           lightResult.xyz += ambientLight + diffuseLight;
    }

//#ifndef GRASS
    vec3 ambientmapped = Remap(gl_LightModel.ambient.xyz * gl_LightModel.ambient.xyz, vec3(ambmin), vec3(1.0));
	  vec3 ambientcon = vec3(mix(gl_LightModel.ambient.xyz * gl_LightModel.ambient.xyz * aoutexp, ambientmapped * aintexp, interiorb));
	
	  float fres = dot(-viewNormal, normalize(viewPos.xyz));

    // skylight
    vec3 skylight = vec3(0.25,0.61,1.0);
	  vec3 overcast = vec3(0.5, 0.5, 0.52);
	  vec4 skyDir = vec4(0.0, 0.0, 1.0, 0.0);
	  skyDir = gl_ModelViewMatrix * skyDir;
	
  	skylight = skylight * skylight;
	  overcast = overcast * overcast;
	  float ambisky = length(clamp(4.0 * gl_LightModel.ambient.xyz - vec3(1.0), 0.0, 1.0));

    vec3 skyterm1 = mix(nightoc * overcast, dayoc * overcast, ambisky);
	  vec3 skyterm2 = daysky * skylight;
    vec3 skyterm3 = lcalcDiffuse(0) * lcalcDiffuse(0) * dayskysun;
	  skylight = 0.10 * mix(skyterm1, skyterm2 + skyterm3,  lcalcDiffuse(0).x * lcalcDiffuse(0).x);
	
	  ambientcon *= ambientcontribution * max(0.0, pow(1.0 - fres,2.0)) + mix(skylight, vec3(1.0), interiorb) * max(0.0, fres);
	
	  skylight *=  clamp((dot(viewNormal.xyz, skyDir.xyz) + 0.6) /((1.0 + 0.6) * (1.0 + 0.6)), 0.0 ,1.0);
    skylight = skylight * (1.0 - interiorb);
    vec3 noskylight = lightResult.xyz;

    lightResult.xyz += ambientcon;
	  lightResult.xyz += skylight;
	  lightResult.xyz *= vec3(vcoff) + vcexp * ambient * ambient;

    if (colorMode == ColorMode_Emission)
        lightResult.xyz += vertexColor.xyz;
    else
        lightResult.xyz += mix(emivnight, emivday, lcalcDiffuse(0).x) * gl_FrontMaterial.emission.xyz;
//#endif

    return lightResult.xyz;
}
#include "lighting_util.glsl"

#ifdef GROUNDCOVER
#define GRASS_WRAP_LIGHTING_COEFF_W 0.6
#define GRASS_WRAP_LIGHTING_COEFF_N 1.5
#define GRASS_BACKLIGHTING_COEFF 1.25
#endif

void perLightSun(out vec3 ambientOut, out vec3 diffuseOut, out vec3 specularOut, vec3 viewPos, vec3 viewNormal, vec4 passparam)
{
    vec3 lightDir = normalize(lcalcPosition(0));
    float lambert = dot(viewNormal.xyz, lightDir);
	float lbounce = max(0.0, -lambert);
	ambientOut = colLoad(lcalcDiffuse(0).xyz) * lbounce * 0.25 * 1.0/4.0;

#ifndef GROUNDCOVER
    lambert = max(lambert, 0.0);
#else
    
	float eyeCosine = dot(normalize(viewPos), viewNormal.xyz);
    /*if (lambert < 0.0)
    {
        lambert = -lambert;
        eyeCosine = -eyeCosine;
    }
    lambert *= clamp(-8.0 * (1.0 - 0.3) * eyeCosine + 1.0, 0.3, 1.0);
	
	*/
	
	float w = GRASS_WRAP_LIGHTING_COEFF_W;
	float n = GRASS_WRAP_LIGHTING_COEFF_N;
	lambert *= -sign(eyeCosine);
	lambert = pow(clamp((lambert + w) / (1.0 + w), 0.0, 1.0), n) * (n + 1.0) / (2.0 * (1.0 + w)) + max(0.0, -1.0 * lambert) * GRASS_BACKLIGHTING_COEFF;
	lambert = max(0.0, lambert);
#endif

    diffuseOut = colLoad(lcalcDiffuse(0).xyz) * lambert;
	
	#if (PER_PIXEL_LIGHTING && @specularMap)
	float microAO = 1.0;
	vec3 kd = vec3(1.0);
	
	float metallic = passparam.x; // maps should have 1 for metals
	float roughness = max(0.015, passparam.y * passparam.y); //linear roughness
	float reflectance = passparam.z; // 0.5 to 0.04 see conversion below
	float AO = passparam.a;
	
	vec3 f0 = vec3(0.16,0.16,0.16) * reflectance * reflectance;

	vec3 l = lightDir;

	vec3 v = normalize(-viewPos.xyz);

	vec3 n = viewNormal;

	float aoFadeTerm = clamp(dot(gl_NormalMatrix * normalize(passNormal), v), 0.0, 1.0);
	AO = mix(1.0, AO, aoFadeTerm);

	microAO = applyAO(AO, dot(l, n));
	
	BRDF(v, l, n, roughness, f0, specularOut, kd);
	diffuseOut *= kd * microAO;
	specularOut *= microAO * colLoad(lcalcDiffuse(0).xyz) * lcalcSpecular(0).xyz;
	#else
	specularOut = vec3(0.0);	
	#endif
}

void perLightPoint(out vec3 ambientOut, out vec3 diffuseOut, out vec3 specularOut, int lightIndex, vec3 viewPos, vec3 viewNormal, vec4 passparam)
{
    vec3 lightPos = lcalcPosition(lightIndex) - viewPos;
    float lightDistance = length(lightPos);

// cull non-FFP point lighting by radius, light is guaranteed to not fall outside this bound with our cutoff
#if !@lightingMethodFFP
    float radius = lcalcRadius(lightIndex);

    if (lightDistance > radius * 2.0)
    {
        ambientOut = vec3(0.0);
        diffuseOut = vec3(0.0);
		specularOut = vec3(0.0);
        return;
    }
#endif

    lightPos = normalize(lightPos);

    float illumination = lcalcIllumination(lightIndex, lightDistance);
    ambientOut = colLoad(lcalcAmbient(lightIndex)) * illumination;
    float lambert = dot(viewNormal.xyz, lightPos) * illumination;

#ifndef GROUNDCOVER
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

    diffuseOut = colLoad(lcalcDiffuse(lightIndex)) * lambert;
	
	#if (PER_PIXEL_LIGHTING && @specularMap)
	float microAO = 1.0;
	vec3 kd = vec3(1.0);
	
	float metallic = passparam.x; // maps should have 1 for metals
	float roughness = max(0.015, passparam.y * passparam.y); //linear roughness
	float reflectance = passparam.z; // 0.5 to 0.04 see conversion below
	float AO = passparam.a;
	
	vec3 f0 = vec3(0.16,0.16,0.16) * reflectance * reflectance;

	vec3 l = lightPos;

	vec3 v = normalize(-viewPos.xyz);

	vec3 n = viewNormal;

	float aoFadeTerm = clamp(dot(gl_NormalMatrix * normalize(passNormal), v), 0.0, 1.0);
	AO = mix(1.0, AO, aoFadeTerm);

	microAO = applyAO(AO, dot(l, n));
	float mask = max(0.0, dot(l, gl_NormalMatrix * normalize(passNormal)));
	mask = clamp(mask * 4.0, 0.0, 1.0);
	BRDF(v, l, n, roughness, f0, specularOut, kd);
	
	diffuseOut *= kd;
	specularOut *= colLoad(lcalcDiffuse(lightIndex)) * illumination * mask;
	#else
	specularOut = vec3(0.0);
	#endif
}

#ifndef GROUNDCOVERVERTEX
uniform mat4 osg_ViewMatrixInverse;
#endif

#if PER_PIXEL_LIGHTING
void doLighting(vec3 viewPos, vec3 viewNormal, vec4 param, float shadowing, out vec3 diffuseLight, out vec3 ambientLight, out vec3 specularLight)
#else
void doLighting(vec3 viewPos, vec3 viewNormal, out vec3 diffuseLight, out vec3 ambientLight, out vec3 shadowDiffuse)
#endif
{	
	vec3 lightpos = (osg_ViewMatrixInverse * vec4(lcalcPosition(0),0)).xyz;
    
	float interiorb = step(0.0, lightpos.y);
	
	#if defined(FORCE_OPAQUE) && FORCE_OPAQUE
	interiorb = 0.0;
	shadowing = 1.0;
	#endif
		
    vec3 ambientOut, diffuseOut, specularOut;
	vec4 passparam = vec4(0.0);
	#if PER_PIXEL_LIGHTING
	passparam = param;
    #endif
	
	perLightSun(ambientOut, diffuseOut, specularOut, viewPos, viewNormal, passparam);
	ambientLight = ambientOut * (1.0 - interiorb);
	vec4 skyDir = vec4(0.0, 0.0, 1.0, 0.0);
	skyDir = gl_ModelViewMatrix * skyDir;
	float skylight = max(0.0, dot(skyDir.xyz, viewNormal));
	skylight = (skylight + 1.0) * 0.5;
	
	skylight = skylight * (1.0 - interiorb) + interiorb;
	
    ambientLight += colLoad(gl_LightModel.ambient.xyz) * skylight;
	
#if PER_PIXEL_LIGHTING
    diffuseLight = diffuseOut * shadowing * (1.0 - interiorb);
	specularLight = specularOut * shadowing * (1.0 - interiorb);
#else
    shadowDiffuse = diffuseOut;
    diffuseLight = vec3(0.0);
#endif

    for (int i = @startLight; i < @endLight; ++i)
    {
#if @lightingMethodUBO
        perLightPoint(ambientOut, diffuseOut, specularOut, PointLightIndex[i], viewPos, viewNormal, passparam);
#else
        perLightPoint(ambientOut, diffuseOut, specularOut, i, viewPos, viewNormal, passparam);
#endif
        ambientLight += ambientOut;
        diffuseLight += diffuseOut;
		#if PER_PIXEL_LIGHTING
		specularLight += specularOut;
		#endif
    }
}

vec3 getSpecular(vec3 viewNormal, vec3 viewDirection, float shininess, vec3 matSpec)
{
    vec3 lightDir = normalize(lcalcPosition(0));
    float NdotL = dot(viewNormal, lightDir);
    if (NdotL <= 0.0)
        return vec3(0.0);
    vec3 halfVec = normalize(lightDir - viewDirection);
    float NdotH = dot(viewNormal, halfVec);
    return pow(max(NdotH, 0.0), max(1e-4, shininess)) * lcalcSpecular(0).xyz * matSpec;
}

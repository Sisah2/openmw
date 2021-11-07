
#ifdef GRASS
#define GRASS_WRAP_LIGHTING_COEFF_W 0.6
#define GRASS_WRAP_LIGHTING_COEFF_N 1.5
#define GRASS_BACKLIGHTING_COEFF 1.25
#endif

void perLightSun(out vec3 ambientOut, out vec3 diffuseOut, vec3 viewPos, vec3 viewNormal)
{
    vec3 lightDir = normalize(lcalcPosition(0));
    float lambert = dot(viewNormal.xyz, lightDir);
	  float lbounce = max(0.0, -lambert);
	  ambientOut = colLoad(lcalcDiffuse(0).xyz) * lbounce * 0.25 * 1.0/4.0;

#ifndef GRASS
    lambert = max(lambert, 0.0);
#else
	float eyeCosine = dot(normalize(viewPos), viewNormal.xyz);
	float w = GRASS_WRAP_LIGHTING_COEFF_W;
	float n = GRASS_WRAP_LIGHTING_COEFF_N;
	lambert *= -sign(eyeCosine);
	lambert = pow(clamp((lambert + w) / (1.0 + w), 0.0, 1.0), n) * (n + 1.0) / (2.0 * (1.0 + w)) + max(0.0, -1.0 * lambert) * GRASS_BACKLIGHTING_COEFF;
	lambert = max(0.0, lambert);
#endif

    diffuseOut = colLoad(lcalcDiffuse(0).xyz) * lambert;
}

void perLightPoint(out vec3 ambientOut, out vec3 diffuseOut, int lightIndex, vec3 viewPos, vec3 viewNormal)
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
        return;
    }
#endif

    lightPos = normalize(lightPos);

    float illumination = lcalcIllumination(lightIndex, lightDistance);
    ambientOut = colLoad(lcalcAmbient(lightIndex)) * illumination;
    float lambert = dot(viewNormal.xyz, lightPos) * illumination;

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

    diffuseOut = colLoad(lcalcDiffuse(lightIndex)) * lambert;
}

#if PER_PIXEL_LIGHTING
void doLighting(vec3 viewPos, vec3 viewNormal, float shadowing, out vec3 diffuseLight, out vec3 ambientLight)
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
		
  vec3 ambientOut, diffuseOut;
	perLightSun(ambientOut, diffuseOut, viewPos, viewNormal);

#if @linearLighting
    ambientLight = ambientOut * (1.0 - interiorb);
	  vec4 skyDir = vec4(0.0, 0.0, 1.0, 0.0);
	  skyDir = gl_ModelViewMatrix * skyDir;
	  float skylight = max(0.0, dot(skyDir.xyz, viewNormal));
	  skylight = (skylight + 1.0) * 0.5;
	  skylight = skylight * (1.0 - interiorb) + interiorb;
    ambientLight += colLoad(gl_LightModel.ambient.xyz) * skylight;
#else
    ambientLight = gl_LightModel.ambient.xyz;
    interiorb = 0.0;
#endif

#if PER_PIXEL_LIGHTING
    diffuseLight = diffuseOut * shadowing * (1.0 - interiorb);
#else
    shadowDiffuse = diffuseOut;
    diffuseLight = vec3(0.0);
#endif

    for (int i = @startLight; i < @endLight; ++i)
    {
        perLightPoint(ambientOut, diffuseOut, i, viewPos, viewNormal);
        ambientLight += ambientOut;
        diffuseLight += diffuseOut;
    }
}
#if @parallax
#define PARALLAX_SCALE 0.04
#define PARALLAX_BIAS -0.02

vec2 getParallaxOffset(vec3 eyeDir, mat3 tbnTranspose, float height, float flipY)
{
    vec3 TSeyeDir = normalize(eyeDir * tbnTranspose);
    return vec2(TSeyeDir.x, TSeyeDir.y * flipY) * ( height * PARALLAX_SCALE + PARALLAX_BIAS );
}

#if defined(TERRAIN_PARALLAX_SOFT_SHADOWS) || defined(OBJECTS_PARALLAX_SOFT_SHADOWS)
float getParallaxShadow(float height, vec2 UV)
{
        vec2 shadowUV = UV;
        float ret = 1.0;
        float h0 = 1.0 - height;
        float h = h0;
        float dist = depth*0.0001;
        float lod1 = 1.0 - step(0.1, dist);

#ifdef TERRAIN
        vec3 point = (osg_ViewMatrixInverse * vec4(0.0,0.0,0.0,1.0)).xyz;
#endif

        for (int i=0; i<MAX_PARAL_LIGHTS; ++i)
        {
            float soften = (i==0) ? 5.0 : 50.0;
            vec3 lightdir = gl_LightSource[i].position.xyz - (passViewPos.xyz * gl_LightSource[i].position.w);
            float lightDistance = max(length(lightdir), 0.1);
            lightdir = normalize(lightdir);
            lightDistance = clamp(1.0 / (gl_LightSource[i].constantAttenuation * 0.1 + 0.01 * gl_LightSource[i].linearAttenuation * lightDistance * lightDistance) - 0.054, 0.0, 1.0);
            lightDistance = clamp(30.0 * lightDistance * lightDistance, 0.0, 1.0);
            vec2 flip = vec2(1.0, -1.0);

#ifdef TERRAIN
            vec3 sundir = normalize((osg_ViewMatrixInverse * vec4(gl_LightSource[0].position.xyz, 1.0)).xyz - point);
            lightdir = (i==0) ? sundir : lightdir;
            flip = (i==0) ? flip * vec2(-1,1) : flip * vec2(1,-1);
            vec2 lDir = flip * (vec2(-lightdir.x, lightdir.y)) * 0.04 * 0.75;
#else
            vec2 lDir = flip * (vec2(lightdir.x, -lightdir.y)) * 0.04 * 0.75;
#endif

            h = min(1.0, 1.0 - texture2D(normalMap, shadowUV + lDir ).w);

            if(lod1 != 0.0)
            {
                h = min( h, 1.0 - texture2D(normalMap, shadowUV + 0.750 * lDir).w);
                h = min( h, 1.0 - texture2D(normalMap, shadowUV + 0.500 * lDir).w);
                h = min( h, 1.0 - texture2D(normalMap, shadowUV + 0.250 * lDir).w);
            }
            ret =  min(ret, 1.0 - ((i==0) ? 1.0 * step(0.01, gl_LightSource[0].diffuse.x) : lightDistance) * clamp((h0 - h) * soften, 0.0, 1.0));
        }
    return ret;
}

#endif
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

vec3 Uncharted2ToneMapping(vec3 color)
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

vec3 SpecialContrast(vec3 x, float suncon) 
{
	//x = pow(x, vec3(0.5 * 2.2));
	vec3 contrasted = x*x*x*(x*(x*6.0 - 15.0) + 10.0);
	x.rgb = mix(x.rgb, contrasted, suncon);
	return x;
}
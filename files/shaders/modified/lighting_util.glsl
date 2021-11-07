#if !@lightingMethodFFP
float quickstep(float x)
{
    x = clamp(x, 0.0, 1.0);
    x = 1.0 - x*x;
    x = 1.0 - x*x;
    return x;
}
#endif

#if @lightingMethodPerObjectUniform

/* Layout:
--------------------------------------- -----------
|  pos_x  |  ambi_r  |  diff_r  |  spec_r         |
|  pos_y  |  ambi_g  |  diff_g  |  spec_g         |
|  pos_z  |  ambi_b  |  diff_b  |  spec_b         |
|  att_c  |  att_l   |  att_q   |  radius/spec_a  |
 --------------------------------------------------
*/
uniform mat4 LightBuffer[@maxLights];
uniform int PointLightCount;

#endif

#if !@lightingMethodFFP
float lcalcRadius(int lightIndex)
{
#if @lightingMethodPerObjectUniform
    return @getLight[lightIndex][3].w;
#else
    return @getLight[lightIndex].attenuation.w;
#endif
}
#endif

float getConstant(int lightIndex)
{
#if @lightingMethodPerObjectUniform
    return @getLight[lightIndex][0].w;
#else
    return @getLight[lightIndex].constantAttenuation;
#endif
}

float getLinear(int lightIndex)
{
#if @lightingMethodPerObjectUniform
    return @getLight[lightIndex][1].w;
#else
    return @getLight[lightIndex].linearAttenuation;
#endif
}

float getQuadratic(int lightIndex)
{
#if @lightingMethodPerObjectUniform
    return @getLight[lightIndex][2].w;
#else
    return @getLight[lightIndex].quadraticAttenuation;
#endif
}

float lcalcIllumination(int lightIndex, float lightDistance)
{
#if defined(LINEAR_LIGHTING) && defined(ATTEN_FIX) && defined(OBJECT) && !@lightingMethodFFP
if(isInterior)
{
    float illumination = clamp(1.0 / (getConstant(lightIndex) * 0.1 + 0.01 * getLinear(lightIndex) * lightDistance * lightDistance) - 0.054, 0.0, 1.0);
    return clamp(illumination * illumination, 0.0, 1.0);
}
else
{
    float illumination = clamp(1.0 / (getConstant(lightIndex) + getLinear(lightIndex) * lightDistance + getQuadratic(lightIndex) * lightDistance * lightDistance), 0.0, 1.0);
    return (illumination * (1.0 - quickstep((lightDistance / lcalcRadius(lightIndex)) - 1.0)));
}
#elif defined(LINEAR_LIGHTING) && !defined(ATTEN_FIX)
    float illumination = clamp(1.0 / (getConstant(lightIndex) * 0.1 + 0.01 * getLinear(lightIndex) * lightDistance * lightDistance) - 0.054, 0.0, 1.0);
    return clamp(illumination * illumination, 0.0, 1.0);
#elif @lightingMethodFFP && defined(LINEAR_LIGHTING) && defined(ATTEN_FIX)
    float illumination = clamp(1.0 / (getConstant(lightIndex) * 0.1 + 0.01 * getLinear(lightIndex) * lightDistance * lightDistance) - 0.054, 0.0, 1.0);
    return clamp(illumination * illumination, 0.0, 1.0);
#elif @lightingMethodFFP
    return clamp(1.0 / (getConstant(lightIndex) + getLinear(lightIndex) * lightDistance + getQuadratic(lightIndex) * lightDistance * lightDistance), 0.0, 1.0);
#else
    float illumination = clamp(1.0 / (getConstant(lightIndex) + getLinear(lightIndex) * lightDistance + getQuadratic(lightIndex) * lightDistance * lightDistance), 0.0, 1.0);
    return (illumination * (1.0 - quickstep((lightDistance / lcalcRadius(lightIndex)) - 1.0)));
#endif
}

vec3 lcalcPosition(int lightIndex)
{
#if @lightingMethodPerObjectUniform
    return @getLight[lightIndex][0].xyz;
#else
    return @getLight[lightIndex].position.xyz;
#endif
}

vec3 lcalcDiffuse(int lightIndex)
{
#if @lightingMethodPerObjectUniform
    return @getLight[lightIndex][2].xyz;
#else
    return @getLight[lightIndex].diffuse.xyz;
#endif
}

vec3 lcalcAmbient(int lightIndex)
{
#if @lightingMethodPerObjectUniform
    return @getLight[lightIndex][1].xyz;
#else
    return @getLight[lightIndex].ambient.xyz;
#endif
}

vec4 lcalcSpecular(int lightIndex)
{
#if @lightingMethodPerObjectUniform
    return @getLight[lightIndex][3];
#else
    return @getLight[lightIndex].specular;
#endif
}

void clampLightingResult(inout vec3 lighting) 
{ 
#if @clamp 
    lighting = clamp(lighting, vec3(0.0), vec3(1.0)); 
#else 
    lighting = max(lighting, 0.0); 
#endif 
}

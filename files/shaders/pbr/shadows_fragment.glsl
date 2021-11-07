#define SHADOWS @shadows_enabled

#if SHADOWS
    uniform float maximumShadowMapDistance;
    uniform float shadowFadeStart;
    @foreach shadow_texture_unit_index @shadow_texture_unit_list
		uniform int shadowTextureUnit@shadow_texture_unit_index;
        uniform sampler2D shadowTexture@shadow_texture_unit_index;
        varying vec4 shadowSpaceCoords@shadow_texture_unit_index;

#if @perspectiveShadowMaps
        varying vec4 shadowRegionCoords@shadow_texture_unit_index;
#endif
    @endforeach
#endif // SHADOWS

#if SHADOWS

/* 
//defaults
const float lightSizeFactor = 0.05;
const float nearPlane = 0.4;
const float maxSearchDistance = 0.01;
const float maxFilterRadius = 0.01;
const int shadowSampleCount = 16;
*/
/*
// initial
const float lightSizeFactor = 0.075;
const float nearPlane = 0.4;
const float maxSearchDistance = 0.01;
const float maxFilterRadius = 0.002;
const int shadowSampleCount = 16;
*/

const float maxFilterRadius = 5.5;
const float maxSearchDistance = 5.0;
const float lightSizeFactor = maxFilterRadius * 0.9;
const float nearPlane = 2.0;
//const int shadowSampleCount = 16;

const int samparray[3] = int[3](8, 4, 2);


float interleavedGradientNoise(vec2 position)
{
    const vec2 magicVector = vec2(0.06711056, 0.00583715);
    const float magicScalar = 52.9829189;
    return fract(magicScalar * fract(dot(position, magicVector)));
}

vec2 vogelDisk(int index, float phi, int cascade)
{
    const float goldenAngle = 2.39996322972865332;
	int samplecount = samparray[cascade];
    float r = sqrt(index + 0.5) / sqrt(samplecount);
    float theta = index * goldenAngle + phi;
    return vec2(r * cos(theta), r * sin(theta));
}

vec2 findShadowOccluders(sampler2D shadowMap, vec3 coords, mat4 eyePlaneMat, float receiver, float phi, int cascade)
{
    float searchDistance = min(maxSearchDistance, lightSizeFactor / receiver * (receiver - nearPlane));
    float scaledDistance = searchDistance / min(2.0, max(1.0, coords.z));
    float depthSum = 0.0;
    int occluderCount = 0;
	int samplecount = samparray[cascade];
    for (int i = 0; i < samplecount; ++i)
    {
        vec4 offset = vec4(vogelDisk(i, phi,cascade) * scaledDistance, 0.0, 0.0);
		offset *= eyePlaneMat;
        float depth = texture2DProj(shadowMap, coords + offset.xyz).r;
        if (depth < receiver)
        {
            ++occluderCount;
            depthSum += depth;
        }
    }
    return vec2(depthSum / occluderCount, occluderCount);
}

float percentageCloserFilter(sampler2D shadowMap, vec3 coords, mat4 eyePlaneMat, float receiver, float filterRadius, float phi, int cascade)
{
    float scaledRadius = filterRadius / min(2.0, max(1.0, coords.z));
    float sum = 0.0;
	int samplecount = samparray[cascade];
    for (int i = 0; i < samplecount; ++i)
    {
        vec4 offset = vec4(vogelDisk(i, phi, cascade) * scaledRadius, 0.0 , 0.0);
		offset *= eyePlaneMat; 
        sum += float(receiver <= texture2DProj(shadowMap, coords + offset.xyz).r);
    }
    return sum / samplecount;
}

float sampleShadow(sampler2D shadowMap, vec4 coords, int cascade, mat4 eyePlaneMat)
{
    float phi = interleavedGradientNoise(gl_FragCoord.xy);
    float receiverDepth = min(coords.z / coords.w, 1.0);
    vec3 coordsProj = coords.xyw;
    vec2 occluders = findShadowOccluders(shadowMap, coordsProj, eyePlaneMat, receiverDepth, phi, cascade);
    if (occluders.y == 0.0)
    {
        return 1.0;
    }

    float meanDepth = occluders.x;
    float penumbra = (receiverDepth - meanDepth) / meanDepth;
    float filterRadius = penumbra * lightSizeFactor * nearPlane / receiverDepth;
    filterRadius = min(filterRadius, maxFilterRadius);
    return percentageCloserFilter(shadowMap, coordsProj, eyePlaneMat, receiverDepth, filterRadius, phi, cascade);
}

#endif

float unshadowedLightRatio(float distance)
{
    float shadowing = 1.0;
 
						
								 
 

 
 
 
 
 
#if SHADOWS
#if @limitShadowMapDistance
    float fade = clamp((distance - shadowFadeStart) / (maximumShadowMapDistance - shadowFadeStart), 0.0, 1.0);
    if (fade == 1.0)
        return shadowing;
#endif
    #if @shadowMapsOverlap
        bool doneShadows = false;
		mat4 eyePlaneMat;
        @foreach shadow_texture_unit_index @shadow_texture_unit_list
		eyePlaneMat = mat4(gl_EyePlaneS[shadowTextureUnit@shadow_texture_unit_index], gl_EyePlaneT[shadowTextureUnit@shadow_texture_unit_index], gl_EyePlaneR[shadowTextureUnit@shadow_texture_unit_index], gl_EyePlaneQ[shadowTextureUnit@shadow_texture_unit_index]);
            if (!doneShadows)
            {
                vec3 shadowXYZ = shadowSpaceCoords@shadow_texture_unit_index.xyz / shadowSpaceCoords@shadow_texture_unit_index.w;
#if @perspectiveShadowMaps
                vec3 shadowRegionXYZ = shadowRegionCoords@shadow_texture_unit_index.xyz / shadowRegionCoords@shadow_texture_unit_index.w;
#endif
                if (all(lessThan(shadowXYZ.xy, vec2(1.0, 1.0))) && all(greaterThan(shadowXYZ.xy, vec2(0.0, 0.0))))
                {
					   
											  
											   
                    shadowing = min(sampleShadow(shadowTexture@shadow_texture_unit_index, shadowSpaceCoords@shadow_texture_unit_index, @shadow_texture_unit_index, eyePlaneMat), shadowing);
										 
	   
	  
 
														  

                    
                    doneShadows = all(lessThan(shadowXYZ, vec3(0.95, 0.95, 1.0))) && all(greaterThan(shadowXYZ, vec3(0.05, 0.05, 0.0)));
#if @perspectiveShadowMaps
                    doneShadows = doneShadows && all(lessThan(shadowRegionXYZ, vec3(1.0, 1.0, 1.0))) && all(greaterThan(shadowRegionXYZ.xy, vec2(-1.0, -1.0)));
#endif
                }
            }
        @endforeach
    #else
		mat4 eyePlaneMat;
        @foreach shadow_texture_unit_index @shadow_texture_unit_list
		eyePlaneMat = mat4(gl_EyePlaneS[shadowTextureUnit@shadow_texture_unit_index], gl_EyePlaneT[shadowTextureUnit@shadow_texture_unit_index], gl_EyePlaneR[shadowTextureUnit@shadow_texture_unit_index], gl_EyePlaneQ[shadowTextureUnit@shadow_texture_unit_index]);
					 
											  
											   
            shadowing = min(sampleShadow(shadowTexture@shadow_texture_unit_index, shadowSpaceCoords@shadow_texture_unit_index, @shadow_texture_unit_index, eyePlaneMat), shadowing);
										 
	   
	  
  
												  
		
        @endforeach
    #endif
#if @limitShadowMapDistance
    shadowing = mix(shadowing, 1.0, fade);
#endif
#endif // SHADOWS
    return shadowing;
}

void applyShadowDebugOverlay()
{
#if SHADOWS && @useShadowDebugOverlay
    bool doneOverlay = false;
    float colourIndex = 0.0;
    @foreach shadow_texture_unit_index @shadow_texture_unit_list
        if (!doneOverlay)
        {
            vec3 shadowXYZ = shadowSpaceCoords@shadow_texture_unit_index.xyz / shadowSpaceCoords@shadow_texture_unit_index.w;
#if @perspectiveShadowMaps
            vec3 shadowRegionXYZ = shadowRegionCoords@shadow_texture_unit_index.xyz / shadowRegionCoords@shadow_texture_unit_index.w;
#endif
            if (all(lessThan(shadowXYZ.xy, vec2(1.0, 1.0))) && all(greaterThan(shadowXYZ.xy, vec2(0.0, 0.0))))
            {
                colourIndex = mod(@shadow_texture_unit_index.0, 3.0);
		        if (colourIndex < 1.0)
			        gl_FragData[0].x += 0.1;
		        else if (colourIndex < 2.0)
			        gl_FragData[0].y += 0.1;
		        else
			        gl_FragData[0].z += 0.1;

                doneOverlay = all(lessThan(shadowXYZ, vec3(0.95, 0.95, 1.0))) && all(greaterThan(shadowXYZ, vec3(0.05, 0.05, 0.0)));
#if @perspectiveShadowMaps
                doneOverlay = doneOverlay && all(lessThan(shadowRegionXYZ.xyz, vec3(1.0, 1.0, 1.0))) && all(greaterThan(shadowRegionXYZ.xy, vec2(-1.0, -1.0)));
#endif
            }
        }
    @endforeach
#endif // SHADOWS
}
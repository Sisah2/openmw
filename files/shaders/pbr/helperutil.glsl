#define LINEAR
#define ACES //OKLAB default tonemapper
//#define DEBUGLIGHTING
//#define PBRDEBUG

float saturate(float x) {
	return clamp(x, 0.0, 1.0);
}

vec2 saturate(vec2 x) {
	return clamp(x, vec2(0.0), vec2(1.0));
}

vec3 saturate(vec3 x) {
	return clamp(x, vec3(0.0), vec3(1.0));
}

float getExposure(float lightEnergy) {
	#ifdef ACES
	return mix(3.14, 3.14, lightEnergy);
	#else
	return mix(4.3, 4.3, lightEnergy);
	#endif
}

float Linear1(float c){return(c<=0.04045)?c/12.92:pow((c+0.055)/1.055,2.4);}
vec3 Linear3(vec3 c){return vec3(Linear1(c.r),Linear1(c.g),Linear1(c.b));}
float Srgb1(float c){return(c<0.0031308?c*12.92:1.055*pow(c,0.41666)-0.055);}
vec3 Srgb3(vec3 c){return vec3(Srgb1(c.r),Srgb1(c.g),Srgb1(c.b));}

vec3 rgb_to_oklab(vec3 c) 
{
    float l = 0.4121656120 * c.r + 0.5362752080 * c.g + 0.0514575653 * c.b;
    float m = 0.2118591070 * c.r + 0.6807189584 * c.g + 0.1074065790 * c.b;
    float s = 0.0883097947 * c.r + 0.2818474174 * c.g + 0.6302613616 * c.b;

    float l_ = pow(l, 1./3.);
    float m_ = pow(m, 1./3.);
    float s_ = pow(s, 1./3.);

    vec3 labResult;
    labResult.x = 0.2104542553*l_ + 0.7936177850*m_ - 0.0040720468*s_;
    labResult.y = 1.9779984951*l_ - 2.4285922050*m_ + 0.4505937099*s_;
    labResult.z = 0.0259040371*l_ + 0.7827717662*m_ - 0.8086757660*s_;
    return labResult;
}

vec3 oklab_to_rgb(vec3 c) 
{
    float l_ = c.x + 0.3963377774 * c.y + 0.2158037573 * c.z;
    float m_ = c.x - 0.1055613458 * c.y - 0.0638541728 * c.z;
    float s_ = c.x - 0.0894841775 * c.y - 1.2914855480 * c.z;

    float l = l_*l_*l_;
    float m = m_*m_*m_;
    float s = s_*s_*s_;

    vec3 rgbResult;
    rgbResult.r = + 4.0767245293*l - 3.3072168827*m + 0.2307590544*s;
    rgbResult.g = - 1.2681437731*l + 2.6093323231*m - 0.3411344290*s;
    rgbResult.b = - 0.0041119885*l - 0.7034763098*m + 1.7068625689*s;
    return rgbResult;
}

vec3 RRTAndODTFit(vec3 v)
{
    vec3 a = v * (v + 0.0245786) - 0.000090537;
    vec3 b = v * (0.983729 * v + 0.4329510) + 0.238081;
    return a / b;
}

vec3 tonemap( vec3 linearRGB )
{
    const float limitHardness = 1.5;
    
    vec3 okl = rgb_to_oklab(linearRGB);
    
    linearRGB = oklab_to_rgb(okl);

    // Try to keep the resulting value within the RGB gamut while
    // preserving chrominance and compensating for negative clipping.
    {
        {
            // Compensate for negative clipping.
            float lumBefore = dot(linearRGB, vec3(0.2126, 0.7152, 0.0722));
            linearRGB = max(vec3(0), linearRGB);
            float lumAfter = dot(linearRGB, vec3(0.2126, 0.7152, 0.0722));
            linearRGB *= lumBefore/lumAfter;
            
            // Keep the resulting value within the RGB gamut.
            linearRGB = linearRGB / pow(pow(linearRGB, vec3(limitHardness)) + vec3(1), vec3(1./limitHardness));
        }
        
        for(int i = 0; i < 2; i++)
        {
            vec3 okl2 = rgb_to_oklab(linearRGB);

            // Control level of L preservation.
            okl2.x = mix(okl2.x, okl.x, 1.0);
             
            float magBefore = length(okl2.yz);
            // Control level of ab preservation.
            okl2.yz = mix(okl2.yz, okl.yz, 0.5);
            float magAfter = length(okl2.yz);
            
            // Uncomment this to only preserve hue.
            okl2.yz *= magBefore/magAfter;

            linearRGB = oklab_to_rgb(okl2);  
  
            {
                // Compensate for negative clipping.
                float lumBefore = dot(linearRGB, vec3(0.2126, 0.7152, 0.0722));
                linearRGB = max(vec3(0), linearRGB);
                float lumAfter = dot(linearRGB, vec3(0.2126, 0.7152, 0.0722));
                linearRGB *= lumBefore/lumAfter;
                
                // Keep the resulting value within the RGB gamut.
                linearRGB = linearRGB / pow(pow(linearRGB, vec3(limitHardness)) + vec3(1), vec3(1./limitHardness));
            }
        }
    }	
    return Srgb3(linearRGB);
}




vec3 LessThan(vec3 f, float value) {
	return vec3(
		(f.x < value) ? 1.0 : 0.0,
		(f.y < value) ? 1.0 : 0.0,
		(f.z < value) ? 1.0 : 0.0);
}

vec3 LinearToSRGB(vec3 rgb) {
	rgb = clamp(rgb, 0.0, 1.0);

	return mix(
		pow(rgb, vec3(1.0 / 2.4)) * 1.055 - 0.055,
		rgb * 12.92,
		LessThan(rgb, 0.0031308)
	);
}

vec3 SRGBToLinear(vec3 rgb) {
	rgb = clamp(rgb, 0.0, 1.0);

	return mix(
		pow(((rgb + 0.055) / 1.055), vec3(2.4)),
		rgb / 12.92,
		LessThan(rgb, 0.04045)
	);
}

vec3 SRGBToLinearApprox(vec3 sRGB) {
	vec3 RGB = sRGB * (sRGB * (sRGB * 0.305306011 + 0.682171111) + 0.012522878);
	return RGB;
}

vec3 colLoad(vec3 sRGB) {
	#ifdef LINEAR
		return SRGBToLinearApprox(sRGB);
	#else
		return sRGB;
	#endif
}

vec3 texLoad(vec3 sRGB) {
	#ifdef LINEAR
		return SRGBToLinear(sRGB);
	#else
		return sRGB;
	#endif
}

vec3 vcolLoad(vec3 vcol) {
	#ifdef LINEAR // hack, makes vcol looks nice in linear lighting
		return sqrt(vcol);
	#else
		return vcol;
	#endif
}

// ACES tone mapping curve fit to go from HDR to LDR
//https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/
vec3 ACESFilm(vec3 x) {
	float a = 2.51;
	float b = 0.03;
	float c = 2.43;
	float d = 0.59;
	float e = 0.14;
	return clamp((x*(a*x + b)) / (x*(c*x + d) + e), 0.0, 1.0);
}

/* PBR Cook Torrance */

#define PI 3.141592653589793238462643383279502884197169

float D_GGX(float NoH, float roughness) {
	float oneMinusNoHSquared = 1.0 - NoH * NoH;
	float a = NoH * roughness;
    float k = roughness / (oneMinusNoHSquared + a * a);
    float d = k * k * (1.0 / PI);
    return d;
}

vec3 F_Schlick(float VoH, vec3 f0) {
	return f0 + (vec3(1.0) - f0) * pow(1.0 - VoH, 5.0);
}

float V_SmithGGXCorrelated(float NoV, float NoL, float roughness) {
    float a2 = roughness * roughness;
 
    float lambdaV = NoL * sqrt((NoV - a2 * NoV) * NoV + a2);
    float lambdaL = NoV * sqrt((NoL - a2 * NoL) * NoL + a2);
	float v = 0.5 / (max(lambdaV + lambdaL, 1e-5));
    return v;
}

float G_Kelemen(float NoL, float NoV, float VoH) {
	float a = NoL * NoV;
	float b = VoH * VoH;
	return a/b;
}

float Fd_Lambert() {
	#ifdef LINEAR
    return 1.0 / PI;
	#else
	return 1.0;
	#endif
}




vec3 toScreen(vec3 color, float exposure) {
	#ifdef LINEAR
		#ifdef PBRDEBUG
			color *= 1.0;
			return color;
		#else
			color *= pow(2.0, exposure);
			#ifdef ACES
				// convert unbounded HDR color range to SDR color range
				color = ACESFilm(color);
	 
				// convert from linear to sRGB for display
				return LinearToSRGB(color);
			#else
				color = RRTAndODTFit(color/1.23);
				return tonemap(color) * 1.23;
			#endif
		#endif
	#else
		return color;
	#endif
}

float applyAO(float ao, float sNoL) {
	float aperture = 2.0 * ao * ao;
	float microShadow = abs(sNoL) + aperture - 1.0;
	return clamp(microShadow, 0.0, 1.0);
}

/* for completeness
	vec3 l = normalize(lcalcPosition(0));
	vec3 v = normalize(-passViewPos.xyz);
	vec3 n = normalize(viewNormal);

	BRDF(v, l, n, roughness, f0, diffuseBRDF, specularBRDF);
*/

//uniform float osg_SimulationTime;


void BRDF(vec3 v, vec3 l, vec3 n, float roughness, vec3 f0, inout vec3 specularBRDF, inout vec3 kd) {
	vec3 h = normalize(v + l);
	float NoV = clamp(dot(n, v), 0.0, 1.0);
	float NoL = clamp(dot(n, l), 0.0, 1.0);
	float NoH = clamp(dot(n, h), 0.0, 1.0);
	float LoH = clamp(dot(l, h), 0.0, 1.0);
	float VoH = clamp(dot(v, h), 0.0, 1.0);
	
	//float f90 = clamp(dot(f0, vec3(50.0 * 0.33)), 0.0, 1.0);
	
	vec3  F = F_Schlick(LoH, f0);
	float G = V_SmithGGXCorrelated(NoV, NoL, roughness);
	float D = D_GGX(NoH, roughness);
	
	
	
	// specular BRDF
	specularBRDF = (D * G) * F * NoL;
	kd -= F;
	
	#ifdef PBRDEBUG // DEBUG
	//specularBRDF = vec3(D) * NoL;
	//specularBRDF = vec3(F) * NoL;
	//specularBRDF = vec3(G) * NoL;
	//specularBRDF = specularBRDF;
	specularBRDF = vec3(0.5) * 3.14;
	#endif
}

/* for completeness
	diffuseBRDF *= lighting * AO;
	gl_FragData[0].xyz = gl_FragData[0].xyz * Fd_Lambert() * diffuseBRDF + specularBRDF * shadowing * SRGBToLinearApprox(lcalcDiffuse(0).xyz);
*/


highp mat3 transpose2(in highp mat3 inMatrix)
{
highp vec3 i0 = inMatrix[0];
highp vec3 i1 = inMatrix[1];
highp vec3 i2 = inMatrix[2];
highp mat3 outMatrix = mat3( vec3(i0.x, i1.x, i2.x), vec3(i0.y, i1.y, i2.y), vec3(i0.z, i1.z, i2.z) );
return outMatrix;
}

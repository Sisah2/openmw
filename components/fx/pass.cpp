#include "pass.hpp"

#include <sstream>
#include <string>
#include <unordered_set>

#include <osg/BindImageTexture>
#include <osg/FrameBufferObject>
#include <osg/Program>
#include <osg/Shader>
#include <osg/State>
#include <osg/StateSet>

#include <components/resource/scenemanager.hpp>
#include <components/sceneutil/lightmanager.hpp>
#include <components/settings/values.hpp>
#include <components/stereo/multiview.hpp>
#include <components/version/version.hpp>

#include "stateupdater.hpp"
#include "technique.hpp"

namespace
{
    constexpr char s_DefaultVertex[] = R"GLSL(
#if OMW_USE_BINDINGS
    omw_In vec2 omw_Vertex;
#endif
omw_Out vec2 omw_TexCoord;

void main()
{
    omw_Position = vec4(omw_Vertex.xy, 0.0, 1.0);
    omw_TexCoord = omw_Position.xy * 0.5 + 0.5;
})GLSL";

    constexpr char s_DefaultVertexMultiview[] = R"GLSL(
layout(num_views = 2) in;
#if OMW_USE_BINDINGS
    omw_In vec2 omw_Vertex;
#endif
omw_Out vec2 omw_TexCoord;

void main()
{
    omw_Position = vec4(omw_Vertex.xy, 0.0, 1.0);
    omw_TexCoord = omw_Position.xy * 0.5 + 0.5;
})GLSL";

}

namespace fx
{
    Pass::Pass(Pass::Type type, Pass::Order order, bool ubo)
        : mCompiled(false)
        , mType(type)
        , mOrder(order)
        , mLegacyGLSL(true)
        , mUBO(ubo)
    {
    }

    std::string Pass::getPassHeader(Technique& technique, std::string_view preamble, bool fragOut)
    {
        std::string header = R"GLSL(
#version @version @profile
@extensions

@uboStruct

#define OMW_API_VERSION @apiVersion
#define OMW_REVERSE_Z @reverseZ
#define OMW_RADIAL_FOG @radialFog
#define OMW_EXPONENTIAL_FOG @exponentialFog
#define OMW_HDR @hdr
#define OMW_NORMALS @normals
#define OMW_NORMFALLBACKDISTANCE @normFallbackDistance
#define OMW_NORMFALLBACKPACKTEXTURE @normFallbackPackTexture
#define OMW_USE_BINDINGS @useBindings
#define OMW_MULTIVIEW @multiview
#define omw_In @in
#define omw_Out @out
#define omw_Position @position
#define omw_Texture1D @texture1D
#define omw_Texture2D @texture2D
#define omw_Texture3D @texture3D
#define omw_Vertex @vertex
#define omw_FragColor @fragColor

@fragBinding

uniform @builtinSampler omw_SamplerLastShader;
uniform @builtinSampler omw_SamplerLastPass;
uniform highp @builtinSampler omw_SamplerDepth;
uniform @builtinSampler omw_SamplerNormals;
uniform @builtinSampler omw_SamplerDistortion;
uniform highp @builtinSampler omw_SamplerPacked;
uniform highp @builtinSampler omw_SamplerNormalsDepth;
uniform highp @builtinSampler omw_SamplerPackedNormals;
uniform highp @builtinSampler omw_SamplerExternalNormals;

uniform vec4 omw_PointLights[@pointLightCount];
uniform int omw_PointLightsCount;

#if OMW_MULTIVIEW
uniform mat4 projectionMatrixMultiView[2];
uniform mat4 invProjectionMatrixMultiView[2];
#endif

int omw_GetPointLightCount()
{
    return omw_PointLightsCount;
}

vec3 omw_GetPointLightWorldPos(int index)
{
    return omw_PointLights[(index * 3)].xyz;
}

vec3 omw_GetPointLightDiffuse(int index)
{
    return omw_PointLights[(index * 3) + 1].xyz;
}

vec3 omw_GetPointLightAttenuation(int index)
{
    return omw_PointLights[(index * 3) + 2].xyz;
}

float omw_GetPointLightRadius(int index)
{
    return omw_PointLights[(index * 3) + 2].w;
}

#if @ubo
    layout(std140) uniform _data { _omw_data omw; };
#else
    uniform _omw_data omw;
#endif


mat4 omw_ProjectionMatrix()
{
#if OMW_MULTIVIEW
    return projectionMatrixMultiView[gl_ViewID_OVR];
#else
    return omw.projectionMatrix;
#endif
}

mat4 omw_InvProjectionMatrix()
{
#if OMW_MULTIVIEW
    return invProjectionMatrixMultiView[gl_ViewID_OVR];
#else
    return omw.invProjectionMatrix;
#endif
}

    float omw_GetDepth(vec2 uv)
    {
#if OMW_MULTIVIEW
        float depth = omw_Texture2D(omw_SamplerDepth, vec3(uv, gl_ViewID_OVR)).r;
#else
        float depth = omw_Texture2D(omw_SamplerDepth, uv).r;
#endif
#if OMW_REVERSE_Z
        return 1.0 - depth;
#else
        return depth;
#endif
    }

    float omw_GetLinearDepth(vec2 uv)
    {
        float far = omw.far;
#if (OMW_REVERSE_Z == 1)
        float depth = omw_GetDepth(uv);
        float dist = omw.near * omw.far / (omw.far + depth * (omw.near - omw.far));
#else
        float depth = omw_GetDepth(uv) * 2.0 - 1.0;
        float dist = 2.0 * omw.near * omw.far / (omw.far + omw.near - depth * (omw.far - omw.near));
#endif

        return dist;
    }

    vec4 omw_GetLastShader(vec2 uv)
    {
#if OMW_MULTIVIEW
        return omw_Texture2D(omw_SamplerLastShader, vec3(uv, gl_ViewID_OVR));
#else
        return omw_Texture2D(omw_SamplerLastShader, uv);
#endif
    }

    vec4 omw_GetLastPass(vec2 uv)
    {
#if OMW_MULTIVIEW
        return omw_Texture2D(omw_SamplerLastPass, vec3(uv, gl_ViewID_OVR));
#else
        return omw_Texture2D(omw_SamplerLastPass, uv);
#endif
    }

    vec3 getPos(vec2 uv, float depth)
    {
        //float depth = omw_GetDepth(uv);
#if (OMW_REVERSE_Z == 1)
        float flippedDepth = 1.0 - depth;
#else
        float flippedDepth = depth * 2.0 - 1.0;
#endif
        vec4 clip_space = vec4(uv * 2.0 - 1.0, flippedDepth, 1.0);
        if (depth == 1.0) {
            vec4 world_space = omw.invViewMatrix * (omw.invProjectionMatrix * clip_space) * vec4(1.0, 1.0, 1.0, 0.0);
            return normalize(world_space.xyz) * 100000000.0;
        }
        else
        {
            vec4 world_space = omw.invViewMatrix * (omw.invProjectionMatrix * clip_space);
            return world_space.xyz / world_space.w;
        }
    }


    vec3 computeNormalImproved(vec2 p, bool worldSpace)
    {
        float W = 1.0/omw.resolution.x;
        float H = 1.0/omw.resolution.y;

        float c0 = omw_GetDepth(p);

        if (c0 == 1.0)
            return vec3(0.0);

        float l2 = omw_GetDepth(p-vec2(2.0*W,0.0));
        float l1 = omw_GetDepth(p-vec2(W,0.0));
        float r1 = omw_GetDepth(p+vec2(W,0.0));
        float r2 = omw_GetDepth(p+vec2(2.0*W,0.0));
        float b2 = omw_GetDepth(p-vec2(0.0,2.0*H));
        float b1 = omw_GetDepth(p-vec2(0.0,H));
        float t1 = omw_GetDepth(p+vec2(0.0,H));
        float t2 = omw_GetDepth(p+vec2(0.0,2.0*H));

        float dl = abs(l1*l2/(2.0*l2-l1)-c0);
        float dr = abs(r1*r2/(2.0*r2-r1)-c0);
        float db = abs(b1*b2/(2.0*b2-b1)-c0);
        float dt = abs(t1*t2/(2.0*t2-t1)-c0);

        vec3 ce = getPos(p,c0);

        vec3 dpdx = (dl<dr) ?  ce-getPos(p-vec2(W,0.0),l1): 
                              -ce+getPos(p+vec2(W,0.0),r1);
        vec3 dpdy = (db<dt) ?  ce-getPos(p-vec2(0.0,H),b1): 
                              -ce+getPos(p+vec2(0.0,H),t1);

        if(worldSpace)
            return normalize(cross(dpdx,dpdy));
        else
            return normalize((vec4(cross(dpdx,dpdy), 1.0) * omw.invViewMatrix).xyz);
    }

    void fromFloat(float encoded, inout float a, inout float b)
    {
        a = floor(encoded) / 63.0;
        b = encoded - floor(encoded);
    }


    void decode(vec4 encoded, inout vec4 scene, inout vec4 normals)
    {
/*
        if (encoded.r < 1.0 || encoded.g < 1.0 || encoded.b < 1.0)
        {
            scene = encoded;
            normals = vec4(1.0, 0.0, 0.0, 1.0);
            return;
        }
*/
        fromFloat(encoded.r, scene.r, normals.r);
        fromFloat(encoded.g, scene.g, normals.g);
        fromFloat(encoded.b, scene.b, normals.b);
        fromFloat(encoded.a, scene.a, normals.a);

        normals = normals * 2.0 - 1.0;
    }

    vec3 getNormals(vec2 uv, bool worldSpace)
    {
#if OMW_NORMFALLBACKPACKTEXTURE
        vec4 scene, normals;
        vec4 encoded = omw_Texture2D(omw_SamplerNormals, uv);

        if (encoded.a == 1.0)
        {
            decode(encoded, scene, normals);
        }
        else
            normals.xyz = omw_Texture2D(omw_SamplerExternalNormals, uv).rgb * 2.0 - 1.0;


/*
        {
            float sceneDepth = omw_GetDepth(uv);
            if (sceneDepth == 1.0)
                return vec3(0.0);

            float normalsDepth = omw_Texture2D(omw_SamplerNormalsDepth, uv).r;

#if (OMW_REVERSE_Z == 1)
            sceneDepth = omw.near * omw.far / (omw.far + sceneDepth * (omw.near - omw.far));
            normalsDepth = omw.near * omw.far / (omw.far + (1.0 - normalsDepth) * (omw.near - omw.far));
#else
            sceneDepth = 2.0 * omw.near * omw.far / (omw.far + omw.near - (sceneDepth * 2.0 - 1.0) * (omw.far - omw.near));
            normalsDepth = 2.0 * omw.near * omw.far / (omw.far + omw.near - (normalsDepth * 2.0 - 1.0) * (omw.far - omw.near));
#endif

//            if (sceneDepth + (sceneDepth * 0.001) > normalsDepth)
            if (encoded.a != 1.0)
                normals.xyz = omw_Texture2D(omw_SamplerNormals, uv).rgb * 2.0 - 1.0;
//            else
//                normals.xyz = computeNormalImproved(uv, false);
        }
*/
        if (worldSpace) 
            normals.xyz = (normals * omw.viewMatrix).xyz;

        return normals.xyz;
#else
        vec3 normal, computedNormal;
        float depth = omw_GetLinearDepth(uv);
        float dist = float(OMW_NORMFALLBACKDISTANCE);

        if (depth >= dist - 1000.0)
            computedNormal = computeNormalImproved(uv, worldSpace);

        if (depth < dist)
        {

#if OMW_MULTIVIEW
            normal = omw_Texture2D(omw_SamplerNormals, vec3(uv, gl_ViewID_OVR)).rgb * 2.0 - 1.0;
#else
            normal = omw_Texture2D(omw_SamplerNormals, uv).rgb * 2.0 - 1.0;
#endif

            if (worldSpace) 
                normal = (vec4(normal, 1.0) * omw.viewMatrix).xyz;

            if (depth > dist - 1000.0)
            {
                float fade = smoothstep(dist - 1000.0, dist, depth);
                normal = mix(normal, computedNormal, fade);
            }
        }
        else if (OMW_NORMFALLBACKDISTANCE != 0)
            normal = computedNormal;

        return normal;
#endif
    }

    vec3 omw_GetNormals(vec2 uv)
    {
        return getNormals(uv, false);
    }

    vec3 omw_GetNormalsWorldSpace(vec2 uv)
    {
        return getNormals(uv, true);
    }

    vec3 omw_GetWorldPosFromUV(vec2 uv)
    {
        float depth = omw_GetDepth(uv);
#if (OMW_REVERSE_Z == 1)
        float flippedDepth = 1.0 - depth;
#else
        float flippedDepth = depth * 2.0 - 1.0;
#endif
        vec4 clip_space = vec4(uv * 2.0 - 1.0, flippedDepth, 1.0);
        vec4 world_space = omw.invViewMatrix * (omw.invProjectionMatrix * clip_space);
        return world_space.xyz / world_space.w;
    }

float omw_EstimateFogCoverageFromUV(vec2 uv)
    {
#if OMW_RADIAL_FOG
        vec3 uvPos = omw_GetWorldPosFromUV(uv);
        float dist = length(uvPos - omw.eyePos.xyz);
#else
        float dist = omw_GetLinearDepth(uv);
#endif
#if OMW_EXPONENTIAL_FOG
        float fogValue = 1.0 - exp(-2.0 * max(0.0, dist - omw.fogNear/2.0) / (omw.fogFar - omw.fogNear/2.0));
#else
        float fogValue = clamp((dist - omw.fogNear) / (omw.fogFar - omw.fogNear), 0.0, 1.0);
#endif

        return fogValue;
    }

#if OMW_HDR
    uniform sampler2D omw_EyeAdaptation;
#endif

    float omw_GetEyeAdaptation()
    {
#if OMW_HDR
        return omw_Texture2D(omw_EyeAdaptation, vec2(0.5, 0.5)).r;
#else
        return 1.0;
#endif
    }
)GLSL";

        std::stringstream extBlock;
        for (const auto& extension : technique.getGLSLExtensions())
            extBlock << "#ifdef " << extension << '\n'
                     << "\t#extension " << extension << ": enable" << '\n'
                     << "#endif" << '\n';

        const std::vector<std::pair<std::string, std::string>> defines
            = { { "@pointLightCount", std::to_string(SceneUtil::PPLightBuffer::sMaxPPLightsArraySize) },
                  { "@apiVersion", std::to_string(Version::getPostprocessingApiRevision()) },
                  { "@version", std::to_string(technique.getGLSLVersion()) },
                  { "@multiview", Stereo::getMultiview() ? "1" : "0" },
                  { "@builtinSampler", Stereo::getMultiview() ? "sampler2DArray" : "sampler2D" },
                  { "@profile", technique.getGLSLProfile() }, { "@extensions", extBlock.str() },
                  { "@uboStruct", StateUpdater::getStructDefinition() }, { "@ubo", mUBO ? "1" : "0" },
                  { "@normals", technique.getNormals() ? "1" : "0" },
                  { "@reverseZ", SceneUtil::AutoDepth::isReversed() ? "1" : "0" },
                  { "@radialFog", Settings::fog().mRadialFog ? "1" : "0" },
                  { "@exponentialFog", Settings::fog().mExponentialFog ? "1" : "0" },
                  { "@hdr", technique.getHDR() ? "1" : "0" }, { "@in", mLegacyGLSL ? "varying" : "in" },
                  { "@out", mLegacyGLSL ? "varying" : "out" }, { "@position", "gl_Position" },
                  { "@texture1D", mLegacyGLSL ? "texture1D" : "texture" },
                  { "@texture2D", mLegacyGLSL ? "texture2D" : "texture" },
                  { "@texture3D", mLegacyGLSL ? "texture3D" : "texture" },
                  { "@vertex", mLegacyGLSL ? "gl_Vertex" : "_omw_Vertex" },
                  { "@fragColor", mLegacyGLSL ? "gl_FragColor" : "_omw_FragColor" },
                  { "@useBindings", mLegacyGLSL ? "0" : "1" },
                  { "@fragBinding", mLegacyGLSL ? "" : "out vec4 omw_FragColor;" },
                  { "@normFallbackPackTexture", (Settings::postProcessing().mNormalsFallbackMode == 2) ? "1" : "0" },
                  { "@normFallbackDistance", std::to_string(Settings::postProcessing().mNormalsFallbackRenderingDistance) } };

        for (const auto& [define, value] : defines)
            for (size_t pos = header.find(define); pos != std::string::npos; pos = header.find(define))
                header.replace(pos, define.size(), value);

        for (const auto& target : mRenderTargets)
            header.append("uniform sampler2D " + std::string(target) + ";");

        for (auto& uniform : technique.getUniformMap())
            if (auto glsl = uniform->getGLSL())
                header.append(glsl.value());

        header.append(preamble);

        return header;
    }

    void Pass::prepareStateSet(osg::StateSet* stateSet, const std::string& name) const
    {
        osg::ref_ptr<osg::Program> program = new osg::Program;
        if (mType == Type::Pixel)
        {
            program->addShader(new osg::Shader(*mVertex));
            program->addShader(new osg::Shader(*mFragment));
        }
        else if (mType == Type::Compute)
        {
            program->addShader(new osg::Shader(*mCompute));
        }

        if (mUBO)
            program->addBindUniformBlock("_data", static_cast<int>(Resource::SceneManager::UBOBinding::PostProcessor));

        program->setName(name);

        if (!mLegacyGLSL)
        {
            program->addBindFragDataLocation("_omw_FragColor", 0);
            program->addBindAttribLocation("_omw_Vertex", 0);
        }

        stateSet->setAttribute(program);

        if (mBlendSource && mBlendDest)
            stateSet->setAttributeAndModes(new osg::BlendFunc(mBlendSource.value(), mBlendDest.value()));

        if (mBlendEq)
            stateSet->setAttributeAndModes(new osg::BlendEquation(mBlendEq.value()));
    }

    void Pass::dirty()
    {
        mVertex = nullptr;
        mFragment = nullptr;
        mCompute = nullptr;
        mCompiled = false;
    }

    void Pass::compile(Technique& technique, std::string_view preamble)
    {
        if (mCompiled)
            return;

        mLegacyGLSL = technique.getGLSLVersion() < 330;

        if (mType == Type::Pixel)
        {
            if (!mVertex)
                mVertex = new osg::Shader(
                    osg::Shader::VERTEX, Stereo::getMultiview() ? s_DefaultVertexMultiview : s_DefaultVertex);

            mVertex->setShaderSource(getPassHeader(technique, preamble).append(mVertex->getShaderSource()));
            mFragment->setShaderSource(getPassHeader(technique, preamble, true).append(mFragment->getShaderSource()));

            mVertex->setName(mName);
            mFragment->setName(mName);
        }
        else if (mType == Type::Compute)
        {
            mCompute->setShaderSource(getPassHeader(technique, preamble).append(mCompute->getShaderSource()));
            mCompute->setName(mName);
        }

        mCompiled = true;
    }

}

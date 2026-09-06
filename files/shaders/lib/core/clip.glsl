#version 330 core

#if defined(GL_ES) && !@useClipDistanceFallback
    #extension GL_EXT_clip_cull_distance: require
#endif

#if defined(GL_ES) || @useClipDistanceFallback
    #define CONDITIONAL_CLIP_DISTANCE
#endif

@foreach clipPlaneIndex @clipPlaneList
    uniform vec4 clipPlane@clipPlaneIndex;
#ifdef CONDITIONAL_CLIP_DISTANCE
    uniform bool clipPlaneEnabled@clipPlaneIndex;
#endif
@endforeach

#if @useClipDistanceFallback
@foreach clipPlaneIndex @clipPlaneList
    out float clip@clipPlaneIndex;
@endforeach
#endif

void applyClipPlanes(vec4 pos)
{
    @foreach clipPlaneIndex @clipPlaneList

#ifdef CONDITIONAL_CLIP_DISTANCE
        if (clipPlaneEnabled@clipPlaneIndex)
#endif
        {
#if @useClipDistanceFallback
            clip@clipPlaneIndex = dot(clipPlane@clipPlaneIndex, pos);
#else
            gl_ClipDistance[@clipPlaneIndex] = dot(clipPlane@clipPlaneIndex, pos);
#endif
        }

    @endforeach
}

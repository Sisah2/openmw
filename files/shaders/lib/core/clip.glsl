@foreach clipPlaneIndex @clipPlaneList
    uniform vec4 clipPlane@clipPlaneIndex;
@endforeach


varying float clip;
uniform bool isReflection;

void applyClipPlanes(vec4 pos)
{
    @foreach clipPlaneIndex @clipPlaneList
        #ifdef GL_EXT_clip_cull_distance
            if (isReflection) gl_ClipDistance[@clipPlaneIndex] = dot(clipPlane@clipPlaneIndex, pos);
        #else
            if (isReflection) clip = dot(clipPlane@clipPlaneIndex, pos);
        #endif
    @endforeach
}

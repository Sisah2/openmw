#version 330 core

@foreach clipPlaneIndex @clipPlaneList
    uniform bool clipPlaneEnabled@clipPlaneIndex;
    in float clip@clipPlaneIndex;
@endforeach

void applyClipPlanes() {
    @foreach clipPlaneIndex @clipPlaneList
        if (clipPlaneEnabled@clipPlaneIndex && clip@clipPlaneIndex < 0.0)
            discard;
    @endforeach
}


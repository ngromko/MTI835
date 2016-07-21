#version 450

layout (binding = 1) uniform sampler2D colorMap;
layout (binding = 2) uniform sampler2D burn;
layout (binding = 3) uniform samplerCubeArray shadowMaps;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 worldPos;
layout (location = 3) in float inHeat;

layout (location = 0) out vec4 outFragColor;

#define EPSILON 0.15

void main() 
{
    vec3 lights[2];
    lights[0] = vec3(5.0f,10.0f,0.0f);
    lights[1] = vec3(5.0f,10.0f,0.0f);
    
    outFragColor = vec4(0.01f);
    
	vec3 N = normalize(inNormal);
    
    vec3 color = texture(colorMap, inUV).rgb;
    vec3 color2 = texture(burn, inUV).rgb;
    color = color*(1.0f-inHeat) + inHeat*color2;
    
    //for(uint i=0;i<2;i++){
    uint i=0;
        // Shadow
        vec3 lightVec = vec3(worldPos - lights[i]);
        vec3 L = normalize(-lightVec);
        float sampledDist = texture(shadowMaps, vec4(lightVec,0)).r;
        float dist = length(lightVec);

        // Check if fragment is in shadow
        if(dist <= sampledDist + EPSILON){
            outFragColor.rgb += max(dot(N,L), 0.3) * color;
        }
    //}  
    //outFragColor.r=sampledDist;
}
  
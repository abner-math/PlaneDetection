uniform mediump vec3 lightPosition;
uniform mediump vec4 lightColor;
uniform mediump mat4 inverseTransposeModelViewMatrix;
uniform sampler2D sampler;

varying mediump vec3 position;
varying mediump vec3 normal;
varying mediump vec2 textureCoord;
varying lowp vec4 color;
varying lowp vec4 ambientColor;
varying lowp vec4 specularColor;
varying mediump float specularExponent;

void main(void)
{
    /*vec3 L = normalize(lightPosition - position);
    vec3 N = normalize(normal);
    vec3 V = normalize(vec3(inverseTransposeModelViewMatrix[3][0],
                inverseTransposeModelViewMatrix[3][1],
                inverseTransposeModelViewMatrix[3][2]) - position);
    vec3 R = reflect(L, N);
    float diffuseCoeff = max(0, dot(L, N));
    vec4 texel = texture2D(sampler, textureCoord);
    vec4 diffuseColor = clamp(texel + color, 0, 1) * diffuseCoeff;
    float specularCoeff = pow(max(0, dot(R, V)), specularExponent);
    vec4 specularColor = specularColor * specularCoeff;
    vec4 finalColor = ambientColor + lightColor * (diffuseColor + specularColor);
    gl_FragColor = finalColor;*/
    gl_FragColor = vec4(0, 0, 0, 1);
}

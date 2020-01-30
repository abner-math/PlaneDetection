uniform mediump mat4 transformationMatrix;
uniform mediump mat4 modelMatrix;
uniform mediump mat4 inverseTransposeModelMatrix;

attribute mediump vec3 positionAttr;
attribute mediump vec3 normalAttr;
attribute mediump vec2 textureCoordAttr;
attribute lowp vec4 colorAttr;
attribute lowp vec4 ambientColorAttr;
attribute lowp vec4 specularColorAttr;
attribute mediump float specularExponentAttr;

varying mediump vec3 position;
varying mediump vec3 normal;
varying mediump vec2 textureCoord;
varying lowp vec4 color;
varying lowp vec4 ambientColor;
varying lowp vec4 specularColor;
varying mediump float specularExponent;

void main(void)
{
    position = (vec4(positionAttr, 1) * modelMatrix).xyz;
    normal = (vec4(normalAttr, 1) * inverseTransposeModelMatrix).xyz;
    textureCoord = textureCoordAttr;
    color = colorAttr;
    ambientColor = ambientColorAttr;
    specularColor = specularColorAttr;
    specularExponent = specularExponentAttr;
    gl_Position = vec4(positionAttr, 1) * transformationMatrix;
}

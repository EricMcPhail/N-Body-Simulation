#pragma once

#include <map>
#include <string>
#include <glm/glm.hpp>
#include <glad/glad.h>

extern class Shader;
extern struct Character;

const std::string DEFAULT_FONT_PATH = "Fonts/Antonio/Antonio-Regular.ttf";

class TextRenderer {
    Shader* shader;
    std::map<GLchar, Character> Characters;
    unsigned int VAO, VBO;
public:
    TextRenderer(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT, std::string font_name = DEFAULT_FONT_PATH);
    ~TextRenderer();
    void RenderText(std::string text, float x, float y, float scale, glm::vec3 color);
};

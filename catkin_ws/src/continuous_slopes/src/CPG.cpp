#include "CPG.h"

#include <vector>

CPG::CPG()
{
h1, h2 = 0.01;
h1_old, h2_old = 0;
h1_output, h2_output = 0;
w11 = 1.5;
w22 = 1.5;
w12 = -0.4;
w21 = 0.4;
bias1 = 0.01;
bias2 = 0.01;
}

std::vector<float> CPG::get_CPG_Output()
{
    h1_old = h1;
    h2_old = h2;

    std::cout<< "CPG1: " << h1 << " CPG2: " <<  h2 << std::endl;
    h1 = (h2_old * w12) + (h1_old * w11);
    h2 = (h1_old * w21) + (h2_old * w22);
    
    output.clear();
    output.push_back(h1);
    output.push_back(h2);

    return output;
}


constexpr int AGENT_NUM = 10;
#define EPISODE_NUM 10000

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <memory>
#include <array>
#include <locale>
#include "config.h"
#include "agent.h"
#include "biwako.h"
#include "mover.h"

using namespace std;


int main() {

    try {
        std::locale::global(std::locale("ja_JP.UTF-8"));
        std::wcout.imbue(std::locale("ja_JP.UTF-8"));
    } catch(const std::exception& e) {
        std::locale::global(std::locale(""));
    }

    cout << "\n<<DSSQ start>>\n\n";

    biwako* init = new biwako();

    unique_ptr<MoveAgents> moveagents_ptr(new MoveAgents("go"));
    unique_ptr<MoveAgents> mover = std::move(moveagents_ptr);
    moveagents_ptr = nullptr;


   /* unique_ptr<Animator> Ani_ptr(new Animator());
    unique_ptr<Animator> Ani = move(Ani_ptr);
    Ani->show();*/

    delete init;
    init = nullptr;

#ifdef _WIN32
    system("pause");
#else
    std::cout << "Press Enter to continue...";
    std::cin.get();
#endif

    return 0;
}
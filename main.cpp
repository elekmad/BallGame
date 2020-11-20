#include "BallGame.h"

int main(void)
{
    BallGame app;
    app.initApp();
    app.getRoot()->startRendering();
    app.closeApp();
    return 0;
}

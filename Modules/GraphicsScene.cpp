#include "GraphicsScene.h"

Qylon::GraphicsScene::GraphicsScene(){
    Pixmap.setZValue(-1);
    addItem(&Pixmap);
}

Qylon::GraphicsScene::~GraphicsScene(){
}

#ifndef GRAPHICSSCENE_H
#define GRAPHICSSCENE_H
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>


namespace Qylon{
class GraphicsScene : public QGraphicsScene{
    Q_OBJECT
public:
    GraphicsScene();
    ~GraphicsScene();
    void setImage(const QImage &image){
        currentImage = image;
        Pixmap.setPixmap(QPixmap::fromImage(image));
    }
    bool isExisting(QGraphicsItem* item){
        auto items =this->items();
        for(int i=0; i<items.size(); ++i){
            if(item == items.at(i)) return true;
        }
        return false;
    }
    const QImage& getImage(){ return currentImage;}
    QGraphicsPixmapItem* getPixmapItem(){ return &Pixmap;}
private:
    QImage currentImage;
    QGraphicsPixmapItem Pixmap;

};
}
#endif // GRAPHICSSCENE_H

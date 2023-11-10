#ifndef IO_H
#define IO_H

namespace Qylon{
class Input{
public:
    virtual ~Input(){}
};
class Output{
public:
    virtual ~Output(){}
};

class ImageInput : public Input{
public:
    ImageInput(){}
};

class TextInput : public Input{
public:
    TextInput(){}
};

class ImageOutput : public Output{
public:
    ImageOutput(){}
};
class TextOutput : public Output{
public:
    TextOutput(){}
};


}


#endif // IO_H

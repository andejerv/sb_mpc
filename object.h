#pragma once

enum object_type
{
    STATIC,
    DYNAMIC
};

class Object
{
    private:
    object_type type;

    public:
        Object(object_type t);
        ~Object();
        
};
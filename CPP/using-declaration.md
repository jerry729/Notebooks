# using declaration

## In namespace and block scope（管理命名空间）

Using-declaration introduces a member of another namespace into current namespace or block scope.

```c++
#include <iostream>
#include <string>
using std::string;
int main()
{
    string str = "Example";
    using std::cout;
    cout << str;
}
```

## In class definition

Using-declaration introduces a member of a base class into the derived class definition, such as to expose a protected member of base as public member of derived. In this case, *nested-name-specifier* must name a base class of the one being defined. If the name is the name of an overloaded member function of the base class, all base class member functions with that name are introduced. If the derived class already has a member with the same name, parameter list, and qualifications, the derived class member hides or overrides (doesn't conflict with) the member that is introduced from the base class.

介绍一个基类成员到子类，可能是基类的protected成员而在子类中给予了public的权限。如果name是一个重载函数，则基类的所有此name的成员函数都被introduced。如果子类已经有此name的成员，则子类成员隐藏或覆盖从基类介绍来的成员。

```cpp
#include <iostream>
struct B {
    virtual void f(int) { std::cout << "B::f\n"; }
    void g(char)        { std::cout << "B::g\n"; }
    void h(int)         { std::cout << "B::h\n"; }
 protected:
    int m; // B::m is protected
    typedef int value_type;
};
 
struct D : B {
    using B::m; // D::m is public
    using B::value_type; // D::value_type is public
 
    using B::f;
    void f(int) { std::cout << "D::f\n"; } // D::f(int) overrides B::f(int)
    using B::g;
    void g(int) { std::cout << "D::g\n"; } // both g(int) and g(char) are visible
                                           // as members of D
    using B::h;
    void h(int) { std::cout << "D::h\n"; } // D::h(int) hides B::h(int)
};
 
int main()
{
    D d;
    B& b = d;
 
//    b.m = 2; // error, B::m is protected
    d.m = 1; // protected B::m is accessible as public D::m
    b.f(1); // calls derived f()
    d.f(1); // calls derived f()
    d.g(1); // calls derived g(int)
    d.g('a'); // calls base g(char)
    b.h(1); // calls base h()
    d.h(1); // calls derived h()
}
```



#### 三种继承方式

三种继承特点

1、public继承**不改变**基类成员的访问权限

2、private继承使得基类所有成员在子类中的访问权限变为**private (基类的private变量在子类中永远无法访问)**

3、protected继承将基类中public成员变为子类的protected成员，其它成员的访问 权限不变。 **(基类的private变量在子类中永远无法访问)**

4、基类中的private成员不受继承方式的影响，**子类永远无权访问**。

#### 虚基类

虚基类 的 作用 
虚基类是指：

```cpp
class  SubClass  :  virtual  public  BaseClass 
```

 中以virtual声明的基类！！由于C++支持多重继承，所以对于一个派生类中有几个直接父类，而几个直接父类中有几个可能分别继承自某一个基类（就是父类的父类），这样在构造最终派生类时，会出现最终派生类中含有多个同一个基类的情况，就会产生二义性的问题（不知道该调用哪个基类的成员变量和函数），为解决此问题，需要使用虚基类，即只对此基类生成一块内存区域，这样最终派生类中就只会含有一个基类了 
**典型的需要用虚基类的情况如下： 
**            A 
           /  \ 
          B    C 
           \  / 
            D 
其中D继承自BC,BC分别继承自A,所以A要分别被BC虚拟继承

```cpp
class   A   { 
    public: 
        void   printA()   {cout<<"this   is   A\n";} 
}; 
class   B:virtual   public   A; 
class   C:virtual   public   A; 
class   D:public   B,public   C;  

```

这样在D构造出来后，它的存储区域中只有一个A，不会有二义性问题 

```cpp
//比如：
D  d=new  D; 
//此时若使用
d.printA();//不会有问题；但若B和C不是虚继承自A，就会有二义性问题
```

 

### Inheriting constructors

If the *using-declaration* refers to a constructor of a direct base of the class being defined (e.g. using Base::Base;), all constructors of that base (ignoring member access) are made visible to overload resolution when initializing the derived class.

If overload resolution selects an inherited constructor, it is accessible if it would be accessible when used to construct an object of the corresponding base class: the accessibility of the using-declaration that introduced it is ignored.

If overload resolution selects one of the inherited constructors when initializing an object of such derived class, then the `Base` subobject from which the constructor was inherited is initialized using the inherited constructor, and all other bases and members of `Derived` are initialized as if by the defaulted default constructor (default member initializers are used if provided, otherwise default initialization takes place). The entire initialization is treated as a single function call: initialization of the parameters of the inherited constructor is sequenced-before initialization of any base or member of the derived object.

```cpp
struct B1 {  B1(int, ...) { } };
struct B2 {  B2(double)   { } };
 
int get();
 
struct D1 : B1 {
  using B1::B1;  // inherits B1(int, ...)
  int x;
  int y = get();
};
 
void test() {
  D1 d(2, 3, 4); // OK: B1 is initialized by calling B1(2, 3, 4),
                 // then d.x is default-initialized (no initialization is performed),
                 // then d.y is initialized by calling get()
  D1 e;          // Error: D1 has no default constructor
}
 
struct D2 : B2 {
  using B2::B2; // inherits B2(double)
  B1 b;
};
 
D2 f(1.0);       // error: B1 has no default constructor
```

```cpp
struct W { W(int); };
struct X : virtual W {
 using W::W;   // inherits W(int)
 X() = delete;
};
struct Y : X {
 using X::X;
};
struct Z : Y, virtual W {
  using Y::Y;
};
Z z(0); // OK: initialization of Y does not invoke default constructor of X
```



If the constructor was inherited from multiple base class subobjects of type B, the program is ill-formed, similar to multiply-inherited non-static member functions:

```cpp
struct A { A(int); };
struct B : A { using A::A; };
struct C1 : B { using B::B; };
struct C2 : B { using B::B; };
 
struct D1 : C1, C2 {
  using C1::C1;
  using C2::C2;
};
D1 d1(0); // ill-formed: constructor inherited from different B base subobjects
 
struct V1 : virtual B { using B::B; };
struct V2 : virtual B { using B::B; };
 
struct D2 : V1, V2 {
  using V1::V1;
  using V2::V2;
};
D2 d2(0); // OK: there is only one B subobject.
          // This initializes the virtual B base class,
          //  which initializes the A base class
          // then initializes the V1 and V2 base classes
          //  as if by a defaulted default constructor
```



As with using-declarations for any other non-static member functions, if an inherited constructor matches the signature of one of the constructors of `Derived`, it is hidden from lookup by the version found in `Derived`. If one of the inherited constructors of `Base` happens to have the signature that matches a copy/move constructor of the `Derived`, it does not prevent implicit generation of `Derived` copy/move constructor (which then hides the inherited version, similar to `using operator=`).

```cpp
struct B1 {   B1(int); };
struct B2 {   B2(int); };
 
struct D2 : B1, B2 {
  using B1::B1;
  using B2::B2;
  D2(int);   // OK: D2::D2(int) hides both B1::B1(int) and B2::B2(int)
};
D2 d2(0);    // calls D2::D2(int)
```



Within a templated class, if a using-declaration refers to a dependent name, it is considered to name a constructor if the *nested-name-specifier* has a terminal name that is the same as the *unqualified-id*.

```cpp
template<class T>
struct A : T {
    using T::T; // OK, inherits constructors of T
};
 
template<class T, class U>
struct B : T, A<U> {
    using A<U>::A; // OK, inherits constructors of A<U>
    using T::A;    // does not inherit constructor of T
                   // even though T may be a specialization of A<>
};
```

### Introducing scoped enumerators

In addition to members of another namespace and members of base classes, using-declaration can also introduce enumerators of enumerations into namespace, block, and class scopes.

A using-declaration can also be used with unscoped enumerators.

```cpp
enum class button { up, down };
struct S {
    using button::up;
    button b = up; // OK
};
 
using button::down;
constexpr button non_up = down; // OK
 
constexpr auto get_button(bool is_up)
{
    using button::up, button::down;
    return is_up ? up : down; // OK
}
 
enum unscoped { val };
using unscoped::val; // OK, though needless
```



#### constexpr

为了使函数获取编译时计算的能力，你必须指定constexpr关键字到这个函数。

```cpp
constexpr int multiply (int x, int y)
{
    return x * y;
}

// 将在编译时计算
const int val = multiply( 10, 10 );
```

除了编译时计算的性能优化，constexpr的另外一个优势是，它允许函数被应用在以前调用宏的所有场合。例如，你想要一个计算数组size的函数，size是10的倍数。如果不用constexpr，你需要创建一个宏或者使用模板，因为你不能用函数的返回值去声明数组的大小。但是用constexpr，你就可以调用一个constexpr函数去声明一个数组。

```cpp
constexpr int getDefaultArraySize (int multiplier)
{
    return 10 * multiplier;
}

int my_array[ getDefaultArraySize( 3 ) ];
```

##### constexpr函数的限制

一个constexpr有一些必须遵循的严格要求：

- 函数中只能有一个return语句（有极少特例）

- 只能调用其它constexpr函数

- 只能使用全局constexpr变量

  

  注意递归并不受限制。但只允许一个返回语句，那如何实现递归呢？可以使用三元运算符（?:)。例如，计算n的阶乘：

  ```cpp
  constexpr int factorial (int n)
  {
      return n > 0 ? n * factorial( n - 1 ) : 1;
  }
  ```

  现在你可以使用factorial(2)，编译器将在编译时计算这个值，这种方式运行更巧妙的计算，与内联截然不同。你无法内联一个递归函数。

  constexpr函数还有那些特点？

  一个constexpr函数，只允许包含一行可执行代码。但允许包含typedefs、 using declaration && directives、静态断言等。

##### constexpr和运行时

一个声明为constexpr的函数同样可以在运行时被调用，当这个函数的参数是非常量的

```cpp
int n;
cin >> n;
factorial( n );

```

##### 编译时使用对象

有一个Circle类：

```cpp
class Circle
{
    public:
    Circle (int x, int y, int radius) : _x( x ), _y( y ), _radius( radius ) {}
    double getArea () const
    {
        return _radius * _radius * 3.1415926;
    }
    private:
        int _x;
        int _y;
        int _radius;
};
```

你希望在编译期构造一个Circle接着算出他的面积。

```cpp
constexpr Circle c( 0, 0, 10 );
constexpr double area = c.getArea();
```

事实证明你可以给Circle类做一些小的修改以完成这件事。首先，我们需要将构造函数声明为constexpr，接着我们需要将getArea函数声明为constexpr。将构造函数声明为constexpr则运行构造函数在编译期运行，只要这个构造函数的参数为常量，且构造函数仅仅包含成员变量的constexpr构造（所以默认构造可以看成constexpr，只要成员变量都有constexpr构造）。

```cpp
class Circle
{
    public:
    constexpr Circle (int x, int y, int radius) : _x( x ), _y( y ), _radius( radius ) {}
    constexpr double getArea ()
    {
        return _radius * _radius * 3.1415926;
    }
    private:
        int _x;
        int _y;
        int _radius;
};
```

##### constexpr vs const

假如你将一个成员函数标记为constexpr，则顺带也将它标记为了const。如果你将一个变量标记为constexpr，则同样它是const的。但相反并不成立，一个const的变量或函数，并不是constexpr的。



## Notes

Only the name explicitly mentioned in the using-declaration is transferred into the declarative scope: in particular, enumerators are not transferred when the enumeration type name is using-declared.

A using-declaration cannot refer to a namespace, to a scoped enumerator (until C++20), to a destructor of a base class or to a specialization of a member template for a user-defined conversion function.

A using-declaration cannot name a member template specialization (template-id is not permitted by the grammar):

```cpp
struct B { template<class T> void f(); };
struct D : B {
      using B::f;      // OK: names a template
//    using B::f<int>; // Error: names a template specialization
      void g() { f<int>(); }
};
```

A using-declaration also can't be used to introduce the name of a dependent member template as a *template-name*(the `template` disambiguator for dependent names is not permitted)

```cpp
template<class X> struct B { template<class T> void f(T); };
template<class Y> struct D : B<Y> {
//  using B<Y>::template f; // Error: disambiguator not allowed
  using B<Y>::f;            // compiles, but f is not a template-name
  void g() {
//    f<int>(0);            // Error: f is not known to be a template name,
                            // so < does not start a template argument list
      f(0);                 // OK
  }   
};
```



## **类型重命名**

作用等同typedef，但是逻辑上更直观。

```cpp
#include <iostream>

using namespace std;

#define DString std::string    //! 不建议使用！

typedef std::string TString;   //! 使用typedef的方式

using Ustring = std::string;   //！使用 using typeName_self = stdtypename;

//更直观
typedef void (tFunc*)(void);
using uFunc = void(*)(void);

int main(int argc, char *argv[])
{

    TString ts("String!");
    Ustring us("Ustring!");    
    string s("sdfdfsd");

　　 cout<<ts<<endl;
    cout<<us<<endl;
    cout<<s<<endl;
    return 0;
}
```


# 复现HDV

## sensor_msgs/Image Message

```
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
```

### step

![img](http://images.cnitblog.com/blog/439761/201410/161551182018972.jpg)

上图三维图像由一个一个名面（第一级）构成，每一个平面由一行一行（第二级）构成，每行由一个一个点（第三级）构成

 同理：二维图像由一行一行（第一级）构成，而每一行又由一个一个点（第二级）构成。

​    Mat中的step[0]就是我们每一个第一级，在内存中占据的字节数量。例如，二维图像中step[0]就是每一行（第一级）在矩阵内存中，占据的字节的数量。也就是说step[i]就是第i+1级在矩阵内存中占据的字节的数量。



## 点云

### 点云数据结构xyz

对于一个三维空间的正则曲面R(u, v), 点(u, v)处的切平面(Ru, Rv)的法向量即为曲面在点(u, v)的法向量。点云是曲面的一个点采样，采样曲面的法向量就是点云的法向量。

![point normal](http://geometryhub.net/images/normal_def.png)

txt格式与xyz格式数据的读取，主要了解其数据结构就OK啦

- txt格式数据一般每行有3列数据，分别为点的 x,y,z坐标，可以以空格、逗号等分隔，读取的时候注意区别就行；

- xyz格式数据一般每行有6列数据，前3列分别为点的 x,y,z坐标，后3列为点的法向量，一般以空格分隔
- 点云数据表示为**至少含有3列**（x, y, z）的N行数组，其中(x, y, z)代表扫描物体表面点的三维空间位置，如果点云信息来自激光雷达传感器等，也可加上强度信息（intensity）和深度信息（depth）等，N代表点云的数量。

![坐标系：图片VS点云](https://img-blog.csdnimg.cn/20190113182302339.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MDAwMDU0MA==,size_16,color_FFFFFF,t_70)

关于图片：

- 图片的坐标值恒为正；
- 图像坐标原点在图片左上角；
- 坐标值为整数
- x轴向右，y轴向下

关于点云坐标：

- 点云的坐标值有正有负；
- 点云值为真实的数值；
- x轴向前，y轴向左，z轴向上；



### PCL pointcloud2

看一下PCLHeader，一个描述点云数据来源的结构体

```text
struct PCLHeader
{
    std::uint32_t seq = 0; // 序列号
    std::uint64_t stamp = 0; // 时间戳，越大越新
    std::string frame_id; // 采样帧id
}
```

描述点云还可以用数据的field来描述，看一下pcd文件的格式，就明白了，每个点的维度是一个field， 要记录这个field的datatype, offset, count.
PointCloud2 是二进制点云结构，你要对保存的数据列一下规则，先存哪个yield，这个yield啥类型，多少个， 这不就知道多少字节了。
先看PCLPointField的定义

```text
 struct PCLPointField
  {
    std::string name; //field的名字 比如x, y, z, range...
    uindex_t offset = 0; // field在点类型的offset
    std::uint8_t datatype = 0; // 基本数据类型
    uindex_t count = 0;  // 基本数据类型的个数， 特征点的filed就可能是一个数组， count就等于数组中元素个数
    enum PointFieldTypes { INT8... /*枚举值，省略了*/ }
  }; 
```

这也就组成了字段信息，（名字，偏移，类型， 个数）
举个例子如果是PointXYZ，总共16字节，有三个field:x, y, z，前四字节存z，中间四字节存y，接下来四字节存x，后面就不管了。也就是说，x-field的name是x， offset是0， datatype是float， count是1，

**PCLPointCloud2**

PCLPointCloud2的定义

```text
struct PCLPointCloud2
{
    ::pcl::PCLHeader header; // 文件头
 index_t height = 0; // 点云高度
 index_t width = 0; // 点云宽度
    std::vector<::pcl::PCLPointField>  fields; // 记录一个点的field信息
 static_assert(BOOST_ENDIAN_BIG_BYTE || BOOST_ENDIAN_LITTLE_BYTE, "unable to determine system endianness");
    std::uint8_t is_bigendian = BOOST_ENDIAN_BIG_BYTE; // 大小端的方式
 index_t point_step = 0; // 一个点的大小，例如PointXYZ 16字节
 index_t row_step = 0; //每一行的字节大小
    std::vector<std::uint8_t> data; // 真实的数据
    std::uint8_t is_dense = 0; // 数据是否合法
  };
```


PCLPointCloud2内部是一个包含Field信息的容器，data是实际的数据；
一个包含两个类型为PointXYZ的点云，
vector存储的大小为32字节, 假设各field的存储顺序是zyx，补位用_代替
那么vector的内容就是 **zyx\_zyx\_**
PCLPointCloud2实现的操作就是合并, 比pointCloud复杂一些



### sensor_msgs/PointCloud2 Message

```
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

```



### sensor_msgs/PointCloud Message

```
# This message holds a collection of 3d points, plus optional additional
# information about each point.

# Time of sensor data acquisition, coordinate frame ID.
Header header

# Array of 3d points. Each Point32 should be interpreted as a 3d point
# in the frame given in the header.
geometry_msgs/Point32[] points

# Each channel should have the same number of elements as points array,
# and the data in each channel should correspond 1:1 with each point.
# Channel names in common practice are listed in ChannelFloat32.msg.
ChannelFloat32[] channels
```



### numpy.c_

- numpy.**c_** *= <numpy.lib.index_tricks.CClass object>*

  Translates slice objects to concatenation along the second axis.This is short-hand for `np.r_['-1,2,0', index expression]`, which is useful because of its common occurrence. In particular, arrays will be stacked along their last axis after being upgraded to at least 2-D with 1’s post-pended to the shape (column vectors made out of 1-D arrays).

```
np.c_[np.array([1,2,3]), np.array([4,5,6])]
array([[1, 4],
       [2, 5],
       [3, 6]])
>>> np.c_[np.array([[1,2,3]]), 0, 0, np.array([[4,5,6]])]
array([[1, 2, 3, ..., 4, 5, 6]])
```

### **二进制大型对象**

（英语：binary large object ，或英语：basic large object，缩写为Blob、BLOB、BLOb），在[数据库管理系统](https://zh.wikipedia.org/wiki/資料庫管理系統)中，将二进制资料存储为一个单一个体的集合。Blob通常是影像、声音或[多媒体](https://zh.wikipedia.org/wiki/多媒體)文件。



### 深/浅拷贝

浅拷贝复制对象本身而不复制对象所引用对象

深拷贝复制对象本身及其引用的对象

赋值不复制只是让指向相同对象



### yaml

清单（数组）

习惯上清单比较常用区块格式（block format）表示，也就是用短杠+空白字符作为起始。

```
 --- # 最喜愛的電影
 - Casablanca
 - North by Northwest
 - Notorious
```

另外还有一种内置格式（inline format）可以选择──用方括号围住，并用逗号+空白区隔（类似[JSON](https://zh.wikipedia.org/wiki/JSON)的语法）。[[6\]](https://zh.wikipedia.org/wiki/YAML#cite_note-6)

```
 --- # 購物清單 
 [milk, pumpkin pie, eggs, juice]
```



字典

键值和资料由冒号及空白字符分开。区块形式（常使用于YAML数据文档中）使用缩进和换行符分隔**key: value**对。内置形式（常使用于YAML数据流中）在大括号中使用逗号+空白字符分隔**key: value**对。

```
 --- # 區塊形式
   name: John Smith
   age: 33
 --- # 內置形式
 {name: John Smith, age: 33}
```



### rospy.Time

ROS has builtin `time` and `duration` primitive types, which rospy provides as the `rospy.Time` and `rospy.Duration` classes, respectively. A `Time` is a specific moment (e.g. "today at 5pm") whereas a `Duration` is a period of time (e.g. "5 hours"). Durations can be negative.

Times and durations have identical representations:

```
int32 secs
int32 nsecs
```

Creating Time instances

There are a variety of ways of creating new `Time` instances in addition to the methods above for getting the current time.

```
rospy.Time(secs=0, nsecs=0)
```

Create a new `Time` instance. `secs` and `nsecs` are optional and default to zero.

```
epoch = rospy.Time() # secs=nsecs=0
t = rospy.Time(10) # t.secs=10
t = rospy.Time(12345, 6789)
```

```
rospy.Time.from_sec(float_secs)
```

Create a new `Time` instance from a float seconds value (same as Python's `time.time()` representation).

```
t = rospy.Time.from_sec(123456.789)
```



### math.atan2

`math.atan2(y, x)`[¶](https://docs.python.org/3/library/math.html?highlight=atan2#math.atan2)

Return `atan(y / x)`, in radians. The result is between `-pi` and `pi`. The vector in the plane from the origin to point `(x, y)` makes this angle **with the positive X axis**. The point of [`atan2()`](https://docs.python.org/3/library/math.html?highlight=atan2#math.atan2) is that the signs of both inputs are known to it, so it can compute the correct quadrant for the angle. For example, `atan(1)` and `atan2(1, 1)` are both `pi/4`, but `atan2(-1, -1)` is `-3*pi/4`.



### sensor_msgs::point_cloud2:: read_points()

```

def sensor_msgs.point_cloud2.read_points	(cloud,
 											field_names = None,
 											skip_nans = False,
 											uvs = [] 
)		
```

```
Read points from a L{sensor_msgs.PointCloud2} message.

@param cloud: The point cloud to read from.
@type  cloud: L{sensor_msgs.PointCloud2}
@param field_names: The names of fields to read. If None, read all fields. [default: None]
@type  field_names: iterable
@param skip_nans: If True, then don't return any point with a NaN value.
@type  skip_nans: bool [default: False]
@param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
@type  uvs: iterable
@return: Generator which yields a list of values for each point.
@rtype:  generator
```



### remap

Sometimes, you may need a message on a specific ROS topic which normally only goes to one set of nodes to also be received by another node. If able, simply tell the new node to subscribe to this other topic. However, you may also do some remapping so that the new node ends up subscribing to `/needed_topic` when it *thinks* it is subscribing to `/different_topic`.

This could be accomplished like so:

```
<remap from="/different_topic" to="/needed_topic"/>
```

Now, when this node *subscribes* to topic `/different_topic`, the remapping makes it *actually subscribe to topic* `/needed_topic`. So, anyone publishing to `/needed_topic` ends up getting their message to this new node as well!

Attributes

**from="original-name"**

Remapped topic: name of the ROS topic that you are remapping FROM.

**to="new-name"**

Target name: name of the ROS topic that you are pointing the `from` topic TO.

Remember: this means that if you remap FROM topic `"A"` TO topic `"B"`, then whenever a node *thinks it is subscribing* to topic `"A"`, it is *actually subscribing* to topic `"B"`, so anyone *publishing to topic `"B"`* will end up getting their message to this node!



## Iterative Closest Point (ICP) algorithm

![img](https://img-blog.csdn.net/20180511150747857)

![img](https://img-blog.csdn.net/20180511150840102)

 刚性变换矩阵中涉及到六个未知数α、β、γ、 tx、ty、tz。要唯一确定这六个未知参数，需要六个线性方程，即至少需要在待匹配点云重叠区域找到3组对应点对，且3组对应点对不能共线，才可以得到这几个未知数的值，进而完成刚性矩阵的参数估计。通常情况下，人们会选择尽可能多的对应点对，进一步提高刚性变换矩阵的参数估计精度



#### ICP 算法的直观想法如下：

- 如果我们知道两幅点云上点的对应关系，那么我们可以用 Least Squares 来求解 R, t 参数；
- 怎么知道点的对应关系呢？如果我们已经知道了一个大概靠谱的 R, t 参数，那么我们可以通过贪心的方式找两幅点云上点的对应关系（直接找距离最近的点作为对应点）



#### ICP 一般算法流程为：

1. 点云预处理

- 滤波、清理数据等

2. 匹配

- 应用上一步求解出的变换，找最近点

3. 加权

- 调整一些对应点对的权重

4. 剔除不合理的对应点对

5. 计算 loss

6. 最小化 loss，求解当前最优变换

7. 回到步骤 2. 进行迭代，直到收敛



#### 初始平移和旋转矩阵

ICP发展了这么多年了，当然有很多的方法来估计初始的R和t了，像PCL给的SampleConsensusInitalAlignment函数以及TransformationEstimationSVD函数都可以得到较好的初始估计



#### 找最近对应点

利用初始 ![[公式]](https://www.zhihu.com/equation?tex=R_0) 、 ![[公式]](https://www.zhihu.com/equation?tex=t_0) 或上一次迭代得到的 ![[公式]](https://www.zhihu.com/equation?tex=R_%7Bk-1%7D) 、 ![[公式]](https://www.zhihu.com/equation?tex=t_%7Bk-1%7D) 对初始点云进行变换，得到一个临时的变换点云，然后用这个点云和目标点云进行比较，找出源点云中每一个点在目标点云中的最近邻点。

如果直接进行比较找最近邻点，需要进行两重循环，计算复杂度为 ![[公式]](https://www.zhihu.com/equation?tex=O%28%7CP_s%7C%E2%8B%85%7CP_t%7C%29) ，这一步会比较耗时，常见的加速方法有：

- 设置距离阈值，当点与点距离小于一定阈值就认为找到了对应点，不用遍历完整个点集；
- 使用 ANN(Approximate Nearest Neighbor) 加速查找，常用的有 KD-tree；KD-tree 建树的计算复杂度为 `O(N log(N))`，查找通常复杂度为 `O(log(N))`（最坏情况下 `O(N)`）



#### 目标函数

在待匹配的两组点云数据的重叠区域内，分别选取两个点集来表示源点集和目标点集，其中P={pi|pi∈R3，i=1,2，……n}为源点集，Q ={qj|qj∈R3，j=1,2，……m}为目标点集，m和n分别代码两个点集的规模。设旋转矩阵为R，平移矩阵为t，用f（R，t）来表示源点集P在变换矩阵（R，t）下与目标点集Q之间的误差。则求解最优变换矩阵的问题就可以转化为求满足min（f（R，t））的最优解（R，t）。

![img](https://img-blog.csdn.net/20180511153017412)

分别在带匹配的目标点云P和源点云Q中，按照一定的约束条件，找到最邻近点（pi，qi），然后计算出最优匹配参数R和t，使得误差函数最小。误差函数为E（R，t）为： ![img](https://img-blog.csdn.net/20180511153352362)

其中n为最邻近点对的个数，pi为目标点云 P 中的一点，qi 为源点云 Q 中与pi对应的最近点，R 为旋转矩阵，t为平移向量。



#### 求解最优变换

对于 point-to-point ICP 问题，求最优变换是有闭形式解（closed-form solution）的，可以借助 SVD 分解来计算。

先给出结论，在已知点的对应关系的情况下，设 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbar+p_s) ， ![[公式]](https://www.zhihu.com/equation?tex=%5Cbar+p_t) 分别表示源点云和目标点云的质心，令 ![[公式]](https://www.zhihu.com/equation?tex=%5Chat+p_s%5Ei+%3D+p_s%5Ei+-+%5Cbar+p_s) ， ![[公式]](https://www.zhihu.com/equation?tex=%5Chat+p_t%5Ei+%3D+p_t%5Ei+-+%5Cbar+p_t) ，令 ![[公式]](https://www.zhihu.com/equation?tex=H+%3D+%5Csum_%7Bi%3D1%7D%5E%7B%7CP_s%7C%7D+%7B%5Chat+p_s%5Ei%7D+%7B%5Chat+p_t%5Ei%7D%5ET) ，这是一个 3x3 矩阵，对 H 进行 SVD 分解得到 ![[公式]](https://www.zhihu.com/equation?tex=H+%3D+U+%5CSigma+V%5ET) ，则 point-to-point ICP 问题最优旋转为：

![[公式]](https://www.zhihu.com/equation?tex=R%5E%7B%5Cast%7D+%3D+V+U%5ET)

最优平移为：

![[公式]](https://www.zhihu.com/equation?tex=t%5E%7B%5Cast%7D+%3D+%5Cbar+p_t+-+R%5E%7B%5Cast%7D+%5Cbar+p_s)

下面分别给出证明。

##### 计算最优平移

令 ![[公式]](https://www.zhihu.com/equation?tex=N+%3D+%7CP_s%7C) ，loss 为 ![[公式]](https://www.zhihu.com/equation?tex=F%28t%29+%3D+%5Csum_%7Bi%3D1%7D%5E%7BN%7D+%7C%7C%28R+%5Ccdot+p_s%5Ei+%2B+t%29+-+p_t%5Ei%7C%7C%5E2) ，对其进行求导，则有：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+%5Cfrac%7B%5Cpartial+F%7D%7B%5Cpartial+t%7D+%26%3D+%5Csum_%7Bi%3D1%7D%5E%7BN%7D+2+%28R+%5Ccdot+p_s%5Ei+%2B+t+-+p_t%5Ei%29+%5C%5C%5C+%26%3D+2nt+%2B+2R%5Csum_%7Bi%3D1%7D%5E%7BN%7Dp_s%5Ei+-+2%5Csum_%7Bi%3D1%7D%5E%7BN%7Dp_t%5Ei+%5Cend%7Baligned%7D)

令导数为 0 ，则有：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+t+%26%3D+%5Cfrac%7B1%7D%7BN%7D+%5Csum_%7Bi%3D1%7D%5E%7BN%7D+p_t%5Ei+-+R+%5Cfrac%7B1%7D%7BN%7D+%5Csum_%7Bi%3D1%7D%5E%7BN%7D+p_s%5Ei+%5C%5C%5C+%26%3D+%5Cbar+p_t+-+R+%5Cbar+p_s+%5Cend%7Baligned%7D)

无论 R 取值如何，根据上式我们都可以求得最优的 t，使得 loss 最小。下面我们来推导最优旋转的计算公式。

##### 计算最优旋转

经过最优平移的推导，我们知道无论旋转如何取值，都可以通过计算点云的质心来得到最优平移，为了下面计算上的简便，我们不妨不考虑平移的影响，先将源点云和目标点云都转换到质心坐标下，这也就是之前令 ![[公式]](https://www.zhihu.com/equation?tex=%5Chat+p_s%5Ei+%3D+p_s%5Ei+-+%5Cbar+p_s) ， ![[公式]](https://www.zhihu.com/equation?tex=%5Chat+p_t%5Ei+%3D+p_t%5Ei+-+%5Cbar+p_t) 的意义。

下面我们用 ![[公式]](https://www.zhihu.com/equation?tex=%5Chat+p_s%5Ei) 和 ![[公式]](https://www.zhihu.com/equation?tex=%5Chat+p_t%5Ei) 进行推导。

不考虑平移，则 loss 可以写成：

![[公式]](https://www.zhihu.com/equation?tex=F%28R%29+%3D+%5Csum_%7Bi%3D1%7D%5E%7BN%7D+%7C%7CR+%5Ccdot+%5Chat+p_s%5Ei+-+%5Chat+p_t%5Ei%7C%7C%5E2)

先化简 ![[公式]](https://www.zhihu.com/equation?tex=%7C%7CR+%5Ccdot+%5Chat+p_s%5Ei+-+%5Chat+p_t%5Ei%7C%7C%5E2) ：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+%7C%7CR+%5Ccdot+%5Chat+p_s%5Ei+-+%5Chat+p_t%5Ei%7C%7C%5E2+%26%3D+%28R+%5Ccdot+%5Chat+p_s%5Ei+-+%5Chat+p_t%5Ei%29%5ET%28R+%5Ccdot+%5Chat+p_s%5Ei+-+%5Chat+p_t%5Ei%29+%5C%5C%5C+%26%3D+%28%7B%5Chat+p_s%5Ei%7D%5ET+R%5ET+-+%7B%5Chat+p_t%5Ei%7D%5ET%29%28R+%5Ccdot+%5Chat+p_s%5Ei+-+%5Chat+p_t%5Ei%29+%5C%5C%5C+%26%3D+%7B%5Chat+p_s%5Ei%7D%5ET+R%5ET+R+%7B%5Chat+p_s%5Ei%7D+-+%7B%5Chat+p_t%5Ei%7D%5ET+R+%7B%5Chat+p_s%5Ei%7D+-+%7B%5Chat+p_s%5Ei%7D%5ET+R%5ET+%7B%5Chat+p_t%5Ei%7D+%2B+%7B%5Chat+p_t%5Ei%7D%5ET+%7B%5Chat+p_t%5Ei%7D+%5C%5C%5C+%26%3D+%7C%7C%7B%5Chat+p_s%5Ei%7D%7C%7C%5E2+%2B+%7C%7C%7B%5Chat+p_t%5Ei%7D%7C%7C%5E2+-+%7B%5Chat+p_t%5Ei%7D%5ET+R+%7B%5Chat+p_s%5Ei%7D+-+%7B%5Chat+p_s%5Ei%7D%5ET+R%5ET+%7B%5Chat+p_t%5Ei%7D+%5C%5C%5C+%26%3D+%7C%7C%7B%5Chat+p_s%5Ei%7D%7C%7C%5E2+%2B+%7C%7C%7B%5Chat+p_t%5Ei%7D%7C%7C%5E2+-+2%7B%5Chat+p_t%5Ei%7D%5ET+R+%7B%5Chat+p_s%5Ei%7D+%5Cend%7Baligned%7D)

这里利用到了 ![[公式]](https://www.zhihu.com/equation?tex=R%5ET+R+%3D+I) 和 ![[公式]](https://www.zhihu.com/equation?tex=%7B%5Chat+p_t%5Ei%7D%5ET+R+%7B%5Chat+p_s%5Ei%7D+%3D+%7B%5Chat+p_s%5Ei%7D%5ET+R%5ET+%7B%5Chat+p_t%5Ei%7D) （标量的转置等于自身）的性质。

由于点的坐标是确定的(和 R 无关)，所以最小化原 loss 等价于求：

![[公式]](https://www.zhihu.com/equation?tex=R%5E%7B%5Cast%7D+%3D+%5Cmathop%7B%5Carg%5Cmin%7D_%7BR%7D+%28-2+%5Csum_%7Bi%3D1%7D%5E%7BN%7D+%7B%5Chat+p_t%5Ei%7D%5ET+R+%7B%5Chat+p_s%5Ei%7D%29)

也即为求：

![[公式]](https://www.zhihu.com/equation?tex=R%5E%7B%5Cast%7D+%3D+%5Cmathop%7B%5Carg%5Cmax%7D_%7BR%7D+%28%5Csum_%7Bi%3D1%7D%5E%7BN%7D+%7B%5Chat+p_t%5Ei%7D%5ET+R+%7B%5Chat+p_s%5Ei%7D%29)

注意到 ![[公式]](https://www.zhihu.com/equation?tex=%5Csum_%7Bi%3D1%7D%5E%7BN%7D+%7B%5Chat+p_t%5Ei%7D%5ET+R+%7B%5Chat+p_s%5Ei%7D+%3D+trace%28P_t%5ET+R+P_s%29) （由矩阵乘法及 trace 的定义可得）

则问题转化为：

![[公式]](https://www.zhihu.com/equation?tex=R%5E%7B%5Cast%7D+%3D+%5Cmathop%7B%5Carg%5Cmax%7D_%7BR%7D+trace%28P_t%5ET+R+P_s%29)

根据 trace 的性质 ![[公式]](https://www.zhihu.com/equation?tex=trace%28AB%29+%3D+trace%28BA%29) ，（这里不要求 A, B 为方阵，只要 A*B 是方阵即可），有：

![[公式]](https://www.zhihu.com/equation?tex=trace%28P_t%5ET+R+P_s%29+%3D+trace%28R+P_s+P_t%5ET%29)

还记得前面定义的矩阵 H 和其 SVD 分解吗？带入上式得到：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+trace%28P_t%5ET+R+P_s%29+%26%3D+trace%28R+P_s+P_t%5ET%29+%5C%5C%5C+%26%3D+trace%28R+H%29+%5C%5C%5C+%26%3D+trace%28R+U+%5CSigma+V%5ET%29+%5C%5C%5C+%26%3D+trace%28%5CSigma+V%5ET+R+U%29+%5Cend%7Baligned%7D)

注意这里 ![[公式]](https://www.zhihu.com/equation?tex=V%2C+U%2C+R) 都是正交矩阵（orthogonal matrices），所以 ![[公式]](https://www.zhihu.com/equation?tex=V%5ET+R+U) 也是正交矩阵。令 ![[公式]](https://www.zhihu.com/equation?tex=M+%3D+V%5ET+R+U+%3D+%5Cleft%5B+%5Cbegin%7Barray%7D%7Bcccc%7D+m_%7B11%7D+%26+m_%7B12%7D+%26+m_%7B13%7D+%5C%5C%5C+m_%7B21%7D+%26+m_%7B22%7D+%26+m_%7B23%7D+%5C%5C%5C+m_%7B31%7D+%26+m_%7B32%7D+%26+m_%7B33%7D+%5Cend%7Barray%7D+%5Cright%5D) ，则有：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+trace%28%5CSigma+V%5ET+R+U%29+%26%3D+trace%28%5CSigma+M%29+%5C%5C%5C+%26+%3D+%5Csigma_1+m_%7B11%7D+%2B+%5Csigma_2+m_%7B22%7D+%2B+%5Csigma_1+m_%7B33%7D+%5Cend%7Baligned%7D)

根据奇异值非负的性质和正交矩阵的性质（正交矩阵中的元素绝对值不大于 1），容易证得只有当 M 为单位阵时 ![[公式]](https://www.zhihu.com/equation?tex=trace%28%5CSigma+M%29) 最大，即：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+V%5ET+R+U+%3D+I+%5C%5C%5C+R+%3D+V+U%5ET+%5Cend%7Baligned%7D)

所以有 ![[公式]](https://www.zhihu.com/equation?tex=R%5E%7B%5Cast%7D+%3D+V+U%5ET) 。

最后还需要进行 Orientation rectification，即验证 ![[公式]](https://www.zhihu.com/equation?tex=R%5E%7B%5Cast%7D+%3D+V+U%5ET) 是不是一个旋转矩阵（检查是否有 ![[公式]](https://www.zhihu.com/equation?tex=%7CR%7C+%3D+1) ），因为存在 ![[公式]](https://www.zhihu.com/equation?tex=%7CR%7C+%3D+-1) 的可能，此时 R 表示的不是旋转而是一个 reflection，所以我们还要给上述优化求解加上一个 ![[公式]](https://www.zhihu.com/equation?tex=%7CR%7C+%3D+1) 的约束。

根据矩阵行列式的性质，以及 U, V 都是正交阵：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+%7CM%7C+%26%3D+%7CV%5ET%7C+%7CU%7C+%7CR%7C+%5C%5C%5C+%26%3D+%7CV%5ET%7C+%7CU%7C+%3D+%5Cpm+1+%5Cend%7Baligned%7D)

如果 ![[公式]](https://www.zhihu.com/equation?tex=%7CV+U%5ET%7C+%3D+1) ，则 ![[公式]](https://www.zhihu.com/equation?tex=%7CM%7C+%3D+1) ， ![[公式]](https://www.zhihu.com/equation?tex=R%5E%7B%5Cast%7D+%3D+V+U%5ET) 已经给出最优旋转；如果 ![[公式]](https://www.zhihu.com/equation?tex=%7CV+U%5ET%7C+%3D+-1) ，则 ![[公式]](https://www.zhihu.com/equation?tex=%7CM%7C+%3D+-1) ，我们需要求解此时的 R，也就是分析 M 应该具有何种形式。具体的讨论请参考[这里](https://link.zhihu.com/?target=http%3A//www.math.pku.edu.cn/teachers/yaoy/Fall2011/arun.pdf)，本文直接给出结论：当 |M|=−1 时，使得 trace(ΣM) 最大的 M 为：

![[公式]](https://www.zhihu.com/equation?tex=M+%3D+%5Cleft%5B+%5Cbegin%7Barray%7D%7Bcccc%7D+1+%26+0+%26+0+%5C%5C%5C+0+%26+1+%26+0+%5C%5C%5C+0+%26+0+%26+-1+%5Cend%7Barray%7D+%5Cright%5D)

综合考虑 ![[公式]](https://www.zhihu.com/equation?tex=%7CM%7C+%3D+1) 和 ![[公式]](https://www.zhihu.com/equation?tex=%7CM%7C+%3D+-1) 两种情况，我们可以得到：

![[公式]](https://www.zhihu.com/equation?tex=R%5E%7B%5Cast%7D+%3D+V+%5Cleft%5B+%5Cbegin%7Barray%7D%7Bcccc%7D+1+%26+0+%26+0+%5C%5C%5C+0+%26+1+%26+0+%5C%5C%5C+0+%26+0+%26+%7CV+U%5ET%7C+%5Cend%7Barray%7D+%5Cright%5D+U%5ET)

至此公式推导完了，简单总结一下求解最优变换的步骤：

1. 计算源点云和目标点云质心；
2. 将源点云和目标点云进行转换到质心坐标系；
3. 计算矩阵 H（形式类似“协方差矩阵”）；
4. 对 H 求 SVD 分解，根据公式求得 ![[公式]](https://www.zhihu.com/equation?tex=R%5E%5Cast) ；
5. 根据公式计算 ![[公式]](https://www.zhihu.com/equation?tex=t%5E%5Cast)

##### 迭代

每一次迭代我们都会得到当前的最优变换参数 ![[公式]](https://www.zhihu.com/equation?tex=R_k%2Ct_k) ，然后将该变换作用于当前源点云；“找最近对应点”和“求解最优变换”这两步不停迭代进行，直到满足迭代终止条件，常用的终止条件有：

- ![[公式]](https://www.zhihu.com/equation?tex=R_k%2Ct_k) 的变化量小于一定值
- loss 变化量小于一定值
- 达到最大迭代次数



## Time Synchronizer

The `TimeSynchronizer` filter synchronizes incoming channels by the timestamps contained in their headers, and outputs them in the form of a single callback that takes the same number of channels.

Suppose you are writing a ROS node that needs to process data from two time synchronized topics. Your program will probably look something like this:

```
import message_filters
from sensor_msgs.msg import Image, CameraInfo

def callback(image, camera_info):
  # Solve all of perception here...

image_sub = message_filters.Subscriber('image', Image)
info_sub = message_filters.Subscriber('camera_info', CameraInfo)

ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
ts.registerCallback(callback)
rospy.spin()
```

### ApproximateTimeSynchronizer

```
class message_filters.ApproximateTimeSynchronizer(fs, queue_size, slop, allow_headerless=False, reset=False)
Approximately synchronizes messages by their timestamps.

```

ApproximateTimeSynchronizer synchronizes incoming message filters by the timestamps contained in their messages’ headers. The API is the same as TimeSynchronizer except for an extra slop parameter in the constructor that defines the delay (in seconds) with which messages can be synchronized. The allow_headerless option specifies whether to allow storing headerless messages with current ROS time instead of timestamp. You should avoid this as much as you can, since the delays are unpredictable.

```
registerCallback(cb, *args)
Register a callback function cb to be called when this filter has output. The filter calls the function cb with a filter-dependent list of arguments, followed by the call-supplied arguments args.
```



## 旋转向量 旋转矩阵 欧拉数 四元数

![img](https://pic3.zhimg.com/80/v2-16060cb3b34ed798427d4b1bfcd3c642_720w.jpg)



### 1. 旋转矩阵与旋转向量

- **旋转矩阵（Rotation Matrix）**

- - 用 9 个量描述旋转的3个自由度，有冗余；
  - 9 个量是有约束的：必须是正交矩阵，且行列式为 1

- **旋转向量（Rotation Vector）**

- - 任意的旋转都可以用一个旋转轴和绕轴的旋转角来描述，简称“轴角”（Axis-Angle）；
  - 旋转向量，是一个三维向量，其方向与旋转轴一致，长度等于旋转角；

旋转向量到旋转矩阵的转换是通过**罗德里格斯公式（Rodrigues's Formula）**，转换公式如下：

![[公式]](https://www.zhihu.com/equation?tex=R+%3D+cos+%5Ctheta+%5C+I+%2B+%281-cos%5Ctheta%29nnx%5E%7BT%7D+%2B+sin%5Ctheta+%5C+nx%5E%7B%5Cwedge%7D+%5C%5C)

其中， ![[公式]](https://www.zhihu.com/equation?tex=n) 为旋转轴单位向量， ![[公式]](https://www.zhihu.com/equation?tex=%5Ctheta) 为旋转角度， ![[公式]](https://www.zhihu.com/equation?tex=%5E%7B%5Cwedge%7D) 为向量到反对称矩阵的运算符。

旋转矩阵到旋转向量的转换过程如下：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Balign%7D+tr%28R%29+%26%3D+cos%5Ctheta+%5Ccdot+tr%28I%29+%2B%281-cos+%5Ctheta%29%5Ccdot+tr%28nn%5E%7BT%7D%29+%2B+sin+%5Ctheta+%5Ccdot+tr%28n%5E%7B%5Cwedge%7D%29+%5C%5C+%26%3D+3cos%5Ctheta+%2B+%281-cos%5Ctheta%29+%5C%5C+%26%3D1%2B2cos%5Ctheta+%5Cend%7Balign%7D+++++++++%5C%5C)

可得转角 ![[公式]](https://www.zhihu.com/equation?tex=%5Ctheta) ：

![[公式]](https://www.zhihu.com/equation?tex=%5Ctheta+%3D+arccos%28%5Cfrac%7Btr%28R%29-1%7D%7B2%7D%29+%5C%5C)

对于转角 ![[公式]](https://www.zhihu.com/equation?tex=n) ，旋转轴上的向量在旋转后不发生改变：

![[公式]](https://www.zhihu.com/equation?tex=Rn%3Dn+%5C%5C)

因此，转轴 ![[公式]](https://www.zhihu.com/equation?tex=n) 是矩阵 ![[公式]](https://www.zhihu.com/equation?tex=R) 特征值 1 对应的特征向量。



```cpp
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

// 1. Eigen 实现

    // 旋转向量转旋转矩阵
    Eigen::Vector3d rvec (r_x, r_y, r_z);     
    double n_norm = rvec.norm();
    Eigen::AngleAxisd rotation_vector (n_norm, rvec/n_norm);
    Eigen::Matrix3d rotm;
    rotm = rotation_vector.toRotationMatrix();

    // 旋转矩阵转旋转向量
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << x_00,x_01,x_02,x_10,x_11,x_12,x_20,x_21,x_22;
    Eigen::AngleAxisd rotation_vector;
    rotation_vector.fromRotationMatrix(rotation_matrix);
 
// 2. OpenCV 实现

    // 旋转向量转旋转矩阵
    cv::Mat rvec = (cv::Mat_<double>(3,1) << r_x, r_y, r_z);
    cv::Mat rotm;
    cv::Rodrigues(rvec, rotm);

    // 旋转矩阵转旋转向量
    cv::Mat rvec;
    cv::Mat rotm = (cv::Mat_<double>(3,3) << x_00,x_01,x_02,x_10,x_11,x_12,x_20,x_21,x_22);
    cv::Rodrigues(rotm, rvec);
```



### 2. 欧拉角

旋转矩阵和旋转向量描述旋转时非常不直观，而欧拉角提供了一种比较直观的表示法：将旋转分解成3次绕不同轴的旋转。由于分解方式有多种，欧拉角也存在不同的定义方法，大致可分为两类：

- 绕固定轴旋转；
- 绕旋转之后的轴旋转

![img](https://pic2.zhimg.com/v2-35da80e428ca5750491ffd2770e187e1_b.jpg)

比较常用的一种欧拉角为 RPY（“偏航 - 俯仰 - 翻转”/ Yaw - Pitch - Roll），等价于 ZYX 轴的旋转：

1. 绕物体的 Z 轴旋转，得到偏航角 Yaw；
2. 绕旋转后的 Y 轴旋转，得到俯仰角 Pitch；
3. 绕旋转后的 X 轴旋转，得到翻转角 Roll。

![img](https://pic1.zhimg.com/80/v2-4ac1fe02ee929bc5b9866356addd95d8_720w.jpg)

欧拉角虽然比较直观，但存在**万向节死锁（Gimbal Lock）**问题：在俯仰角为正负90° 时，第一次旋转和第三次旋转将使用一个轴，使得系统丢失一个自由度，会发生奇异。因此，欧拉角不适于插值和迭代。

假设我们要把 3D 物体旋转到某个想要的方向时，我们按照先以 x 轴为旋转轴旋转 ![[公式]](https://www.zhihu.com/equation?tex=%5Calpha) 角，然后绕 y 轴旋转 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbeta) 角，最后绕 z 轴旋转 ![[公式]](https://www.zhihu.com/equation?tex=%5Cgamma) 角这样的顺序实现（即按欧拉角来表示旋转），用旋转矩阵来表示的话，即

![[公式]](https://www.zhihu.com/equation?tex=R+%3D+%5Cleft%5B%5Cbegin%7Bmatrix%7D%5Ccos%7B%5Cleft+%28%5Cbeta+%5Cright+%29%7D+%5Ccos%7B%5Cleft+%28%5Cgamma+%5Cright+%29%7D+%26+%5Csin%7B%5Cleft+%28%5Calpha+%5Cright+%29%7D+%5Csin%7B%5Cleft+%28%5Cbeta+%5Cright+%29%7D+%5Ccos%7B%5Cleft+%28%5Cgamma+%5Cright+%29%7D+-+%5Csin%7B%5Cleft+%28%5Cgamma+%5Cright+%29%7D+%5Ccos%7B%5Cleft+%28%5Calpha+%5Cright+%29%7D+%26+%5Csin%7B%5Cleft+%28%5Calpha+%5Cright+%29%7D+%5Csin%7B%5Cleft+%28%5Cgamma+%5Cright+%29%7D+%2B+%5Csin%7B%5Cleft+%28%5Cbeta+%5Cright+%29%7D+%5Ccos%7B%5Cleft+%28%5Calpha+%5Cright+%29%7D+%5Ccos%7B%5Cleft+%28%5Cgamma+%5Cright+%29%7D%5C%5C%5Csin%7B%5Cleft+%28%5Cgamma+%5Cright+%29%7D+%5Ccos%7B%5Cleft+%28%5Cbeta+%5Cright+%29%7D+%26+%5Csin%7B%5Cleft+%28%5Calpha+%5Cright+%29%7D+%5Csin%7B%5Cleft+%28%5Cbeta+%5Cright+%29%7D+%5Csin%7B%5Cleft+%28%5Cgamma+%5Cright+%29%7D+%2B+%5Ccos%7B%5Cleft+%28%5Calpha+%5Cright+%29%7D+%5Ccos%7B%5Cleft+%28%5Cgamma+%5Cright+%29%7D+%26+-+%5Csin%7B%5Cleft+%28%5Calpha+%5Cright+%29%7D+%5Ccos%7B%5Cleft+%28%5Cgamma+%5Cright+%29%7D+%2B+%5Csin%7B%5Cleft+%28%5Cbeta+%5Cright+%29%7D+%5Csin%7B%5Cleft+%28%5Cgamma+%5Cright+%29%7D+%5Ccos%7B%5Cleft+%28%5Calpha+%5Cright+%29%7D%5C%5C-+%5Csin%7B%5Cleft+%28%5Cbeta+%5Cright+%29%7D+%26+%5Csin%7B%5Cleft+%28%5Calpha+%5Cright+%29%7D+%5Ccos%7B%5Cleft+%28%5Cbeta+%5Cright+%29%7D+%26+%5Ccos%7B%5Cleft+%28%5Calpha+%5Cright+%29%7D+%5Ccos%7B%5Cleft+%28%5Cbeta+%5Cright+%29%7D%5Cend%7Bmatrix%7D%5Cright%5D)

这里需要注意的是

> 我们虽说是用欧拉角来表示旋转，但是最终还是要构造对应的**转换矩阵**，并将该矩阵应用到 3D 物体的顶点上才最终达成旋转效果。

当我们的物体的初始方向就是已经绕 y 轴旋转了负 90 度，即 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbeta) 角取 ![[公式]](https://www.zhihu.com/equation?tex=-%5Cfrac%7B%5Cpi%7D%7B2%7D) ，那么上面的旋转矩阵将会变成

![[公式]](https://www.zhihu.com/equation?tex=+R+%3D+%5Cleft%5B%5Cbegin%7Bmatrix%7D0+%26+-+%5Csin%28%5Calpha+%2B+%5Cgamma%29+%5C+%26+-+%5Ccos%28%5Calpha+%2B+%5Cgamma%29%5C%5C0+%26+%5Ccos%28%5Calpha+%2B+%5Cgamma%29+%26+-+%5Csin%28%5Calpha+%2B+%5Cgamma%29+%5C%5C1+%26+0+%26+0%5Cend%7Bmatrix%7D%5Cright%5D)

我们注意到 ![[公式]](https://www.zhihu.com/equation?tex=%5Calpha) 角和 ![[公式]](https://www.zhihu.com/equation?tex=%5Cgamma) 角在矩阵元素里是成对出现的，也就是说改变 ![[公式]](https://www.zhihu.com/equation?tex=%5Calpha) 角和改变 ![[公式]](https://www.zhihu.com/equation?tex=%5Cgamma) 角对旋转矩阵的影响是一样的。要实现某个旋转原本我们有 3 个变量可以调整，即 ![[公式]](https://www.zhihu.com/equation?tex=%5Calpha) 角、 ![[公式]](https://www.zhihu.com/equation?tex=%5Cbeta) 角和 ![[公式]](https://www.zhihu.com/equation?tex=%5Cgamma) 角，现在因为![[公式]](https://www.zhihu.com/equation?tex=%5Calpha) 角和 ![[公式]](https://www.zhihu.com/equation?tex=%5Cgamma) 角是等效的，变成只剩下 2 个变量可以调整了。换句话说，我们原本有 3 个自由度，现在变成了 2 个自由度，这就是所谓的**万向节死锁**。

![img](https://pic4.zhimg.com/v2-c59686dbabc1683a9b5d7c68df8178cf_b.jpg)

翻转 Roll

![img](https://pic3.zhimg.com/v2-5ecf4094f573e1ced6a7225bd6ed028a_b.jpg)

俯仰 Pitch

![img](https://pic3.zhimg.com/v2-29ea182b7b4733d0702395b1f34662ca_b.jpg)

偏航 Yaw

![img](https://pic2.zhimg.com/v2-cabe060d9bebb30d976c7035164bcc75_b.jpg)

万向节死锁，只有两个自由度，有奇异性



#### 旋转矩阵->欧拉角

![image-20211017225952731](C:\Users\frank\AppData\Roaming\Typora\typora-user-images\image-20211017225952731.png)



![image-20211017230024516](C:\Users\frank\AppData\Roaming\Typora\typora-user-images\image-20211017230024516.png)



参考一篇NASA的关于姿态描述的技术报告[1]的Appendix-A6和[5]，我们可以用**旋转矩阵元素的相乘、相除、反三角函数等操作去“凑”出欧拉角**。[5]给出了从XYZ顺规提取欧拉角的方法、步骤、思路，[1]则给出了全部12种顺规的欧拉角提取公式，但是没有给一些细节注意事项。所以总结一下，根据[1]、[5]、[7]《Real Time Rendering 3rd Edition》4.2.2和自己的推导，从ZXY顺规旋转矩阵提取欧拉角的公式是（[1]原文下标似乎有点小问题）：

- Y axis yaw angle:

![[公式]](https://www.zhihu.com/equation?tex=%5Calpha%3Datan2%28sin%5Calpha+cos%5Cbeta%2Ccos%5Calpha+cos%5Cbeta%29%3Datan2%28m_%7B13%7D%2Cm_%7B33%7D%29)

- X axis pitch angle:

![[公式]](https://www.zhihu.com/equation?tex=%5Cbeta%3Darcsin%28sin%5Cbeta%29%3Darcsin%28-m_%7B23%7D%29)

- Z axis roll angle:

![[公式]](https://www.zhihu.com/equation?tex=%5Cgamma%3Datan2%28cos%5Cbeta+sin%5Cgamma%2Ccos%5Cbeta+cos%5Cgamma%29%3Datan2%28m_%7B21%7D%2Cm_%7B22%7D%29)

.

注意到一点，注意到矩阵的每一个元素都是pitch angle ![[公式]](https://www.zhihu.com/equation?tex=%5Cbeta) 的函数…所以当 ![[公式]](https://www.zhihu.com/equation?tex=m_%7B23%7D%3D-sin%5Cbeta%3D%5Cpm1) 即 ![[公式]](https://www.zhihu.com/equation?tex=cos%5Cbeta%3D0) 的时候，这时候其他的欧拉角提取表达式就凉凉了（分子分母都是0, arctan和atan2都没有意义了）….其实pitch angle ![[公式]](https://www.zhihu.com/equation?tex=%5Cbeta%3D%5Cpm+%5Cpi%2F2) 恰好就是Gimbal Lock的位置。在Gimbal Lock的时候，旋转矩阵会退化为：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D++R%28%5Calpha%2C%5Cbeta%2C%5Cgamma%29%26%3D+%5Cleft%5B+%5Cbegin%7Bmatrix%7D+c_1c_3%5Cpm+s_1s_3+%26+%5Cpm+c_3s_1-c_1s_3+%26+0+%5C%5C+0+%26+0+%26+%5Cpm+1+%5C%5C+%5Cpm+c_1s_3-s_1c_3+%26+s_1s_3+%5Cpm+c_1c_3+%26+0+%5Cend%7Bmatrix%7D%5Cright%5D%5C%5C++%26%3D%5Cleft%5B+%5Cbegin%7Bmatrix%7D+cos%28%5Calpha+%5Cpm+%5Cgamma%29+%26+sin%28%5Calpha+%5Cpm+%5Cgamma%29+%26+0+%5C%5C+0+%26+0+%26+%5Cpm+1+%5C%5C+-sin%28%5Calpha+%5Cpm+%5Cgamma%29+%26+-cos%28%5Calpha+%5Cpm+%5Cgamma%29+%26+0+%5Cend%7Bmatrix%7D%5Cright%5D++%5Cend%7Baligned%7D)

.

那么要进一步处理万向节死锁的corner case就需要分两种情况：

- ![[公式]](https://www.zhihu.com/equation?tex=%5Cbeta%3D-%5Cpi%2F2) ，此时 ![[公式]](https://www.zhihu.com/equation?tex=+sin%5Cbeta%3D-1%2C+cos%5Cbeta%3D0)

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D++R%28%5Calpha%2C%5Cbeta%2C%5Cgamma%29%26%3D+%5Cleft%5B+%5Cbegin%7Bmatrix%7D+c_1c_3-+s_1s_3+%26+-c_3s_1-c_1s_3+%26+0+%5C%5C+0+%26+0+%26+%5Cpm+1+%5C%5C+-+c_1s_3-s_1c_3+%26+s_1s_3+-+c_1c_3+%26+0+%5Cend%7Bmatrix%7D%5Cright%5D%5C%5C++%26%3D%5Cleft%5B+%5Cbegin%7Bmatrix%7D+cos%28%5Calpha+%2B+%5Cgamma%29+%26+-sin%28%5Calpha+%2B+%5Cgamma%29+%26+0+%5C%5C+0+%26+0+%26+1+%5C%5C+-sin%28%5Calpha+%2B+%5Cgamma%29+%26+-cos%28%5Calpha+%2B+%5Cgamma%29+%26+0+%5Cend%7Bmatrix%7D%5Cright%5D++%5Cend%7Baligned%7D)

![[公式]](https://www.zhihu.com/equation?tex=%5CRightarrow+%5Calpha%2B%5Cgamma+%3Datan2%28-m_%7B12%7D%2Cm_%7B11%7D%29)

其中要给![[公式]](https://www.zhihu.com/equation?tex=%5Calpha)或者 ![[公式]](https://www.zhihu.com/equation?tex=%5Cgamma)其中一个欧拉角赋值，另外一个就按等式计算出来。

.

- ![[公式]](https://www.zhihu.com/equation?tex=%5Cbeta%3D%5Cpi%2F2) ，此时 ![[公式]](https://www.zhihu.com/equation?tex=+sin%5Cbeta%3D1%2C+cos%5Cbeta%3D0)

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D++R%28%5Calpha%2C%5Cbeta%2C%5Cgamma%29%26%3D+%5Cleft%5B+%5Cbegin%7Bmatrix%7D+c_1c_3%2B+s_1s_3+%26+c_3s_1-c_1s_3+%26+0+%5C%5C+0+%26+0+%26+-1+%5C%5C++c_1s_3-s_1c_3+%26+s_1s_3+%2B+c_1c_3+%26+0+%5Cend%7Bmatrix%7D%5Cright%5D%5C%5C++%26%3D%5Cleft%5B+%5Cbegin%7Bmatrix%7D+cos%28%5Calpha+-+%5Cgamma%29+%26+sin%28%5Calpha+-+%5Cgamma%29+%26+0+%5C%5C+0+%26+0+%26+-1+%5C%5C+-sin%28%5Calpha+-+%5Cgamma%29+%26+cos%28%5Calpha+-+%5Cgamma%29+%26+0+%5Cend%7Bmatrix%7D%5Cright%5D++%5Cend%7Baligned%7D)

![[公式]](https://www.zhihu.com/equation?tex=%5CRightarrow+%5Calpha+-+%5Cgamma+%3Datan2%28m_%7B12%7D%2Cm_%7B11%7D%29)

同样的，要给![[公式]](https://www.zhihu.com/equation?tex=%5Calpha)或者 ![[公式]](https://www.zhihu.com/equation?tex=%5Cgamma)其中一个欧拉角赋值，另外一个就按等式计算出来。

.

从旋转矩阵提取欧拉角的公式跟欧拉角顺规的选取有关，因为旋转矩阵的元素会略有不同，但是思路都是一样的，就是**根据旋转矩阵的解析表达式+反三角函数凑出来**。	


![R =R_z(\phi)R_y(\theta)R_x(\psi)=\begin{bmatrix} cos\theta cos\phi & sin\psi sin\theta cos\phi - cos\psi sin\theta & cos \psi sin\theta cos\phi +sin\psi sin\phi \\cos\theta sin\phi & sin\psi sin\theta sin\phi+cos\psi cos\phi & cos\psi sin\theta sin\phi -sin\psi cos\theta \\ -sin\theta & sin\psi cos\theta & cos\psi cos\theta\end{bmatrix}](https://math.jianshu.com/math?formula=R%20%3DR_z(%5Cphi)R_y(%5Ctheta)R_x(%5Cpsi)%3D%5Cbegin%7Bbmatrix%7D%20cos%5Ctheta%20cos%5Cphi%20%26%20sin%5Cpsi%20sin%5Ctheta%20cos%5Cphi%20-%20cos%5Cpsi%20sin%5Ctheta%20%26%20cos%20%5Cpsi%20sin%5Ctheta%20cos%5Cphi%20%2Bsin%5Cpsi%20sin%5Cphi%20%5C%5Ccos%5Ctheta%20sin%5Cphi%20%26%20sin%5Cpsi%20sin%5Ctheta%20sin%5Cphi%2Bcos%5Cpsi%20cos%5Cphi%20%26%20cos%5Cpsi%20sin%5Ctheta%20sin%5Cphi%20-sin%5Cpsi%20cos%5Ctheta%20%5C%5C%20-sin%5Ctheta%20%26%20sin%5Cpsi%20cos%5Ctheta%20%26%20cos%5Cpsi%20cos%5Ctheta%5Cend%7Bbmatrix%7D)
则可以如下表示欧拉角：
![\theta_x=atan2(r_{32},r_{33})](https://math.jianshu.com/math?formula=%5Ctheta_x%3Datan2(r_%7B32%7D%2Cr_%7B33%7D))
![\theta_y=atan2(-r_{31},\sqrt{r_{32}^2+r_{33}^2})](https://math.jianshu.com/math?formula=%5Ctheta_y%3Datan2(-r_%7B31%7D%2C%5Csqrt%7Br_%7B32%7D%5E2%2Br_%7B33%7D%5E2%7D))
![\theta_z=atan2(r_{21},r_{11})](https://math.jianshu.com/math?formula=%5Ctheta_z%3Datan2(r_%7B21%7D%2Cr_%7B11%7D))
以下代码用来实现旋转矩阵和欧拉角之间的相互变换。

```cpp
Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
    assert(isRotationMatirx(R));
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2( R(2,1), R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2( R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return {x, y, z};
}
```

```
# Checks if a matrix is a valid rotation matrix.

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).

def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

```



#### 内旋外旋

**内旋和外旋：**

- 每次旋转是绕固定轴（一个固定参考系，比如世界坐标系）旋转，称为外旋。
- 每次旋转是绕自身旋转之后的轴旋转，称为内旋。
- 下图说明了内旋和外旋的区别。

![img](https://pic3.zhimg.com/80/v2-54aa5fb93eb991ef96e47c63fc0b807a_720w.jpg)



###  4. 四元数

四元数（Quaternion）既是紧凑的，又没有奇异性，缺点是不够直观，运算比较复杂。

四元数 ![[公式]](https://www.zhihu.com/equation?tex=q) 有一个实部和三个虚部，如下所示：

![[公式]](https://www.zhihu.com/equation?tex=q+%3D+q_0+%2B+q_1+i+%2B+q_2+j+%2B+q_3+k+%5C%5C)

三个虚部 ![[公式]](https://www.zhihu.com/equation?tex=i%2C+j%2C+k) 满足以下关系式：

![[公式]](https://www.zhihu.com/equation?tex=i%5E%7B2%7D+%3D+j%5E%7B2%7D++%3D+k%5E%7B2%7D+%3D+-1++%5C%5C+ij+%3D+k%2C+%5Cquad+ji+%3D+-k+%5C%5C+jk%3Di%2C+%5Cquad+kj%3D-i+%5C%5C+ki%3Dj%2C+%5Cquad+ik%3D-j+%5C%5C)

（1）四元数与旋转向量

- 已知旋转向量（绕单位向量 ![[公式]](https://www.zhihu.com/equation?tex=n+%3D+%5Bn_x%2C+n_y%2C+n_z%5D%5ET) 旋转角度 ![[公式]](https://www.zhihu.com/equation?tex=%5Ctheta) ），求四元数

![[公式]](https://www.zhihu.com/equation?tex=q+%3D+%5Bcos%5Cfrac%7B%5Ctheta%7D%7B2%7D%2C+%5C+n_x+sin+%5Cfrac%7B%5Ctheta%7D%7B2%7D%2C+%5C+n_y+sin%5Cfrac%7B%5Ctheta%7D%7B2%7D%2C+%5C+n_z+sin%5Cfrac%7B%5Ctheta%7D%7B2%7D+%5D%5ET+%5C%5C)

- 已知四元数，求旋转向量

![[公式]](https://www.zhihu.com/equation?tex=%5Ctheta+%3D+2+%5C+arccos+%5C++q_0++%5C%5C+%5Bn_x%2C+%5C+n_y%2C+%5C+n_z%5D%5ET+%3D+%5Bq_1%2C+%5C+q_2%2C+%5C+q_3%5D%5ET+%2F+sin+%5Cfrac%7B%5Ctheta%7D%7B2%7D)

（2）四元数与旋转矩阵

- 已知四元数，求旋转矩阵

![img](https://pic3.zhimg.com/80/v2-94a9864878cf4c2d33715b71f578d4de_720w.jpg)

或者：

![[公式]](https://www.zhihu.com/equation?tex=q+%3D+%5Bw%2C+x%2C+y%2Cz%5D)

![[公式]](https://www.zhihu.com/equation?tex=R+%3D+%5Cleft%5B%5Cbegin%7Barray%7D%7Bccc%7D+1-2+y%5E%7B2%7D-2+z%5E%7B2%7D+%26+2+x+y%2B2+w+z+%26+2+x+z-2+w+y+%5C%5C+2+x+y-2+w+z+%26+1-2+x%5E%7B2%7D-2+z%5E%7B2%7D+%26+2+y+z%2B2+w+x+%5C%5C+2+x+z%2B2+w+y+%26+2+y+z-2+w+x+%26+1-2+x%5E%7B2%7D-2+y%5E%7B2%7D+%5Cend%7Barray%7D%5Cright%5D)

- 已知旋转矩阵，求四元数

![img](https://pic2.zhimg.com/80/v2-ab1fd5ab960f2bc02ffa3d68048769d9_720w.jpg)



## numpy 数组索引

这个:

```
x[:, 1]
```


表示“沿第一个轴获取 `x` 的所有索引，但沿第二个轴仅获取索引 1”。

这个:

```
x[:][1]
```


表示“沿第一个轴获取 `x` 的所有索引(因此所有 `x` )，然后沿 **第一个** 结果的轴获取索引 1”。您正在申请 `1`到错误的轴。
`x[1][2]`和 `x[1, 2]`只是等价的，因为使用整数索引数组会将所有剩余的轴移向形状的前面，因此 `x[1]` 的第一个轴是 `x` 的第二个轴.这根本不能概括；您应该几乎总是使用逗号而不是多个索引步骤。



## point_cloud2.py

```
Functions
def sensor_msgs.point_cloud2._get_struct_fmt (is_bigendian, fields, field_names=None)
def sensor_msgs.point_cloud2.create_cloud (header, fields, points)
def sensor_msgs.point_cloud2.create_cloud_xyz32 (header, points)
def sensor_msgs.point_cloud2.read_points (cloud, field_names=None, skip_nans=False, uvs=[])
def sensor_msgs.point_cloud2.read_points_list (cloud, field_names=None, skip_nans=False, uvs=[])
```



### publisher的queue_size

自从 hydro 以后，推荐使用新的 asynchronous ，异步的，发布机制，它更像是roscpp的做法。
为了使用新的发布机制，the keyword queue_size 就要传给 subscribe() ，它定义了在消息丢失之前的最大队列大小。
当publish()被调用时，会把串行化了的数据发到每一个subscriber，连接会异步地发生自不同的线程，但串行化仍将同步地发生。这就使得只有出现连接问题的subscriber会接收不到新的消息。
如果你发布消息的速度快于rospy能发送出去的速度，rospy会把老的messges丢掉。
注意，传输层上还会有个操作系统级的序列，比如TCP/IP发送buffer.

queue_size Omitted
If the keyword argument is omitted, None is passed or for Groovy and older ROS distributions the publishing is handled synchronously. As of Indigo not passing the keyword argument queue_size will result in a warning being printed to the console.

queue_size None
Not recommended. Publishing is handled synchronously which means that one blocking subscriber will block all publishing. As of Indigo passing None will result in a warning being printed to the console.

queue_size Zero
While a value of 0 means an infinite queue, this can be dangerous since the memory usage can grow infinitely and is therefore not recommended.

queue_size One, Two, Three
If your system is not overloaded you could argue that a queued message should be picked up by the dispatcher thread within a tenth of a second. So a queue size of 1 / 2 / 3 would be absolutely fine when using 10 Hz.

Setting the queue_size to 1 is a valid approach if you want to make sure that a new published value will always prevent any older not yet sent values to be dropped. This is good for, say, a sensor that only cares about the latest measurement. e.g. never send older measurements if a newer one exists.

queue_size Ten or More
An example of when to use a large queue size, such as 10 or greater, is user interface messages (e.g. digital_io, a push button status) that would benefit from a larger queue_size to prevent missing a change in value. Another example is when you want to record all published values including the ones which would be dropped when publishing with a high rate / small queue size.



## 旋转矩阵不可交换

![preview](https://pica.zhimg.com/42d485afda852242df2209c77cfab11f_r.jpg?source=1940ef5c)



```
 ->	R1, R2, P1, P2, Q, validPixROI1, validPixROI2
```


# 📘 Essential C++ for ROS 2: Revision Notes

## 1. Memory Management (The "Smart" Way)

In the old days of C++, you had to manually allocate memory (`new`) and remember to delete it (`delete`). If you forgot, your program leaked memory. If you deleted it too soon, your program crashed.

ROS 2 uses **Smart Pointers** to handle this automatically.

### A. `std::shared_ptr`

Think of a shared pointer as a **Smart Trash Can**.

- **Normal Pointer:** You throw trash on the floor. You must remember to pick it up later.
- **Shared Pointer:** You put trash in the smart can. The can has a counter.
- When you hold the can, the counter is 1.
- If you give a copy to your friend, the counter becomes 2.
- When your friend leaves, the counter drops to 1.
- When you leave, the counter drops to 0.
- **Magic:** When the counter hits 0, the can _automatically_ destroys itself and the trash.

### B. `std::make_shared<T>()`

This is the "Factory" that builds the Smart Trash Can.

- **The Rule:** Always use `std::make_shared` instead of `new`. It's faster and safer.

**Example:**

```cpp
// ❌ Old/Bad Way (Raw Pointer)
MyMessage* msg = new MyMessage();
// ... if I forget 'delete msg', memory leak!

// ✅ ROS 2 Way (Smart Pointer)
// "Create a shared pointer of type MyMessage"
auto msg = std::make_shared<MyMessage>();
// No need to delete! It handles itself.

```

---

## 2. Callbacks & Functional Magic

ROS 2 relies heavily on **Callbacks**. A callback is just a function you give to ROS saying, _"Hey, hold this function and run it when a message arrives."_

### A. `std::bind`

Imagine you have a specific machine (an object) with a button (a function) on it.

- The ROS Timer wants to press a button, but it doesn't know _which_ machine to walk up to.
- `std::bind` is like glue. It glues the **Machine** (`this`) to the **Button** (`function`) so you can hand the whole package to ROS.

**Syntax:**

```cpp
// "Glue the timer_callback function... to THIS specific node."
std::bind(&MinimalPublisher::timer_callback, this)

```

### B. Placeholders (`std::placeholders::_1`)

This is used when the callback expects data (like a Subscriber).

- **The Problem:** You are gluing the function _now_, but the message data isn't available yet. It will arrive _later_.
- **The Solution:** You put a sticky note labeled `_1` in the function's input slot. It basically says: _"Leave this slot open. When the real message arrives later, plug it in here."_

**Example:**

```cpp
// "Glue topic_callback to THIS node, and put the incoming message in slot #1"
std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1)

```

### C. Lambdas (`[](){}`)

A Lambda is a **Disposable Function**. Instead of writing a proper named function at the top of your file, you write a quick, throwaway function right where you need it.

- **`[]` (Capture):** This is the backpack. It lists variables from the outside world that the lambda needs to carry inside to do its job.
- **`()` (Args):** The inputs (just like a normal function).
- **`{}` (Body):** The code to run.

**Example:**

```cpp
// A simple lambda that prints "Hello"
auto my_lambda = []() {
    std::cout << "Hello!";
};
my_lambda(); // Runs the code

```

---

## 3. Object-Oriented Basics

ROS 2 nodes are almost always Classes.

### A. Inheritance (`public rclcpp::Node`)

This is the **"Is-A"** relationship.

- When you write `class MyRobot : public rclcpp::Node`, you are saying: _"MyRobot **IS A** Node."_
- Because it **is** a Node, it automatically gets all the superpowers of a ROS Node (like talking to topics, parameters, and time) without you writing that code yourself.

### B. The `this` Pointer

- **What is it?** It's a pointer that says **"Me"** or **"Myself"**.
- **Why use it?** When you are inside a class, and you want to use a function that belongs to _that specific instance_ of the class, you use `this`.

**Example:**

```cpp
// "Create a publisher... attached to ME."
this->create_publisher<String>("topic", 10);

```

### C. Member Initializer Lists

This is the stuff after the colon `:` in a constructor. It's the **VIP Entrance** for setting up variables.

- **Body Assignment (Slow):** You create an empty box, _then_ you put a toy in it.
- **Initializer List (Fast):** You create the box _with the toy already inside_.

**Example:**

```cpp
// ✅ Good: Initializer List
MyClass() : count_(0) {}

// ❌ Okay, but slower: Body Assignment
MyClass() {
    count_ = 0;
}

```

---

## 4. Syntax Essentials

### A. `auto`

The **Lazy (but Smart) Programmer's Tool**.

- Instead of typing out long, complicated types, you tell the compiler: _"You figure it out."_
- The compiler looks at the right side of the `=` sign to deduce the type.

**Example:**

```cpp
// ❌ Painful to type
std::shared_ptr<std_msgs::msg::String> msg = std::make_shared<std_msgs::msg::String>();

// ✅ Easy
auto msg = std::make_shared<std_msgs::msg::String>();

```

### B. `const` & References (`&`)

This is about efficiency.

- **Passing by Value:** `void func(String s)`
- Makes a **photocopy** of the string. Slow for big data.

- **Passing by Reference:** `void func(String & s)`
- Passes the **original** string. Fast, but dangerous (function might change it).

- **Const Reference:** `void func(const String & s)`
- Passes the **original** string (Fast), but puts it in a clear glass box so the function can **look but not touch** (Safe).
- _Always use this for callbacks!_

### C. Namespaces (`namespace`)

Imagine a school where two students are named "John". Confusion!

- **Namespaces** are like "Classrooms".
- `ClassA::John` and `ClassB::John` are now clearly different.
- **`std::`**: The classroom for Standard C++ tools.
- **`rclcpp::`**: The classroom for ROS 2 tools.
- **`::`**: This symbol (Scope Resolution Operator) is just how you open the classroom door.

---

## 5. Program Structure

### A. `main(int argc, char * argv[])`

- **The Entry Point:** Every C++ program starts here.
- **`argc` (Argument Count):** How many words did you type in the command line?
- **`argv` (Argument Vector):** An array (list) of the actual words you typed.
- **Why in ROS?** ROS uses this to sniff out special ROS commands (like remapping topics) before your program even starts.

### B. Strings: `std::string` vs `std_msgs`

- **`std::string`**: The clay you mold in your hands. Use this for math, logic, and combining words (`"Hello" + " World"`).
- **`std_msgs::msg::String`**: The shipping box. You cannot do math on a box. You must put your clay (`std::string`) _inside_ the box (`.data`) to ship it to another node.

**Conversion:**

```cpp
int number = 42;
// You can't just add a number to a string. You must convert it first.
std::string text = "Answer: " + std::to_string(number);

```

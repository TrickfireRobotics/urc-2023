# **Getting Started**

Welcome to the software subteam! We hope that you will stick around and learn quite a bit of things about robots! However, before you can do that, you should review the resources given in this document to familarize yourself. 


## **Libraries/Tools/Technologies that We Utilize**
In this particular codebase, we use the following libraries/tools/technologies
- Docker (To standardize software development and deployment)
- ROS2 (Robot Operating System)
- Python
- C++ 
- HTML/Javascript/CSS/Front end frameworks
- Git/Github

## **What is Docker?**
Docker allows us to create "virtual machines" called **containers** without actually running a proper virtual machine. To create a **container**, we use a **docker image**. This [Stackoverflow](https://stackoverflow.com/questions/28089344/docker-what-is-it-and-what-is-the-purpose) post explains it quite well.

## **What is ROS?**
The **Robot Operating System (ROS)** is not an operating system, but instead a collection of libraries and tools that allow software developers to write code more focused on robots. We use ROS2 Humble.

The core component of ROS are **Nodes** and its **Observer Pattern** in the form of **Subscribers/Publishers**. Each **node** is essentially its own little program doing its own thing. If you have two, or perhaps more, **nodes** that you want to communicate, how do you do that? Here comes the **publisher** - it allows a **node** to publish ("yell out") data on a certain topic (like a frequency for a radio). To receive this data, a node can subscribe ("listen to") to a topic published by another node.

Here is a good video that talks more about ROS and its other communication protocols

[![IMAGE ALT TEXT](https://img.youtube.com/vi/7TVWlADXwRw/0.jpg)](https://www.youtube.com/watch?v=7TVWlADXwRw "Video Title")

## **What is Git/Github?**
Git is an open source version control software that makes working on the same codebase much easier. 

The core idea of Git is that code lives on different "branches" where code differs as developers work on it. There is always a "master/main" branch which represents code that is ready to be deployed into action. 

As you work on your code in a **feature branch**, you will "save" changes represented as a **Git commit**. After you have committed enough changes/additions, you then can open up a **Pull Request** in order for other developers to review your code before it then gets merged into the master/main branch. After it is merged, congratulations your feature is now live. 

Github is a website that allows us to have a Git repository that lives on the cloud. This makes sharing remote repositories very easy. 

If you have no clue what commands to use for Git, go [here](https://learngitbranching.js.org/?locale=en_US) to learn! Just a couple of lessons are enough to understand how it works.

## **Documentation**
All of the documentation for this codebase is stored inside this `docs` folder. Here, you will find a markdown file for each ROS Node (packge/folder) under the `src` folder that explains its workings in more detail. You will also find some other general documentation in there.

## **What Should I do Now?**
Checkout the [code_overview.md](./code_overview.md) to understand how the codebase is structured. Also take a look at the (formatting.md)[./formatting.md] document to make sure that your code is readable and well maintained. 

After reading this document, you should follow the instructions for installing on [Windows,](./install_on_windows.md), installing on [Mac](./installing_on_mac.md), or installing on [Linux](./install_on_linux.md)


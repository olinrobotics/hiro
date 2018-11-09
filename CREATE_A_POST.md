# How to create a post

This is the instruction on how to create a post for the [IRL project page](https://olinrobotics.github.io/irl/projects.html).

If you have any questions, email Khang Vu @ minhkhang.vu@students.olin.edu
## 1. Overview
In order to create a post, you'll need two things:
- A `README.md` file in a [Markdown format](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet). This file needs to have a **front matter block** on the top for Jekyll to generate the web page. _(see Section 2a)_   
- Any images used in your README file.

The basic idea is Jekyll will use Markdown files to generate webpages. However, the Markdown file might be a little bit different.

## 2. Basic preparations

### a. How to prepare your README file
Copy this [raw Markdown template](https://raw.githubusercontent.com/olinrobotics/irl/gh-pages/_posts/README_TEMPLATE.md) and save it as a markdown file (with a file extension `.md`)

Read this before you edit that file:
* The file include a **front matter block** on the top. This code block tells Jekyll how to generate the site. 
```markdown
---
title: Project Name
date: 2018-10-15
categories: [Fall 2018]
excerpt: "This is a short description about your project to show in the project listing page."
thumbnail: "/assets/images/thumbnail.jpg"
coverImgURL: "/assets/images/thumbnail.jpg"
published: true
---
```  
* `thumbnail`, `coverImgURL`, and `published` are optional fields. You don't have to include them if you don't have thumbnail/cover images. `published` indicates whether you want to show or hide the post.
* The app will sort the posts based on their published date. Therefore, you should put `date: yyyy-mm-dd` correctly. To make it easy, you could choose the Olin Demo date as the published date. (Don't put the date in the future because the post can't be published in the future)
* Note the way we show images: The path `{{ site.baseurl }}/assets/images/` is important to show your image on the website. You wouldn't be able to see your image in the Markdown file, however.
```markdown
![Alternative text]({{ site.baseurl }}/assets/images/thumbnail.jpg "Your image title")
```
* Note the way we show Youtube video: you can just copy the embedded code block from Youtube and paste it here
```html
<iframe width="560" height="315" 
        src="https://www.youtube.com/embed/DGbLDu6VkR0" 
        frameborder="0" 
        allow="autoplay; encrypted-media" allowfullscreen>
</iframe>
```

### b. Prepare image files
If possible, convert all your image files into `.jpg` and reduce the image size. If you use Photoshop, simply open a image file in Photoshop and go to `File > Export > Save for Web (Legacy)`, choose `.jpg` medium size.

## 3. Publish your post
After gathering these things, you'll have 2 options:
### Option 1: upload the post by yourself (Recommended)
If you choose this option, you will 
- have a chance to view and test your page locally!
- learn how our page works!!
- save the page maintainers a ton of time!!!

#### Step 1: Install requirements
Open Terminal and check whether you have Ruby 2.1.0 or higher installed:
```console
$ ruby --version
ruby 2.X.X
```
If you don't have Ruby installed, install [Ruby 2.1.0 or higher](https://www.ruby-lang.org/en/documentation/installation/).

Then follow more instructions to install Jekyll here (look at the **Guide section**): [https://jekyllrb.com/docs/installation/](https://jekyllrb.com/docs/installation/)

#### Step 2: Check out gh-pages branch
Simply clone IRL and checkout `gh-pages`:
```console
$ git clone https://github.com/olinrobotics/irl.git
$ cd irl
$ git checkout gh-pages
```
If you already cloned IRL, make sure you have the lasted version of `gh-pages` by running:
```console
$ git pull origin gh-pages
```
#### Step 3: branch off from `gh-pages`
You should have your own branch when working on your post because you will not be able to push directly to `gh-pages`, and you'll need to create a pull request later on in **Step 7**.

To branch off, simply type:
```
git checkout -b {branch_name}
```
- `-b` indicates that you're creating a new branch and checking out that branch immediately.
- `{branch_name}` is the name of your branch. You should name it like `projectname-docs`.

#### Step 4: Build your local Jekyll site
First, let's install Jekyll using Bundler. In the `irl` folder in your local machine, run
```console
$ bundle install
```
Now, it's time to run IRL page locally:
```console
$ bundle exec jekyll serve
```
You'll get something like this:
```console
Configuration file: /home/username/Git/irl/_config.yml
            Source: /home/username/Git/irl
       Destination: /home/username/Git/irl/_site
 Incremental build: disabled. Enable with --incremental
      Generating... 
        Pagination: Pagination is enabled, but I couldn't find an index.html page to use as the pagination template. Skipping pagination.
                    done in 0.052 seconds.
 Auto-regeneration: enabled for '/home/username/Git/irl'
    Server address: http://127.0.0.1:4000/irl/
  Server running... press ctrl-c to stop.
```
Depending your local machine, you might have different server address. Mine was `http://127.0.0.1:4000/irl/`. Open that link in the browser (or just `Ctrl + Click` on it).

#### Step 5: Add your post
Now, it's time to handle your post. 

First, rename your `README.md` file to this format `yyyy-mm-dd-projectname.md`. Below are some examples
```console
2018-12-15-pushcup.md
2018-12-15-push-cup.md
2018-12-15-sudoku.md
2018-12-15-sudoku-game.md
...
```
Because the app will sort the posts based on their published date, it is recommended that you put `yyyy-mm-dd` correctly. To make it easy, you could choose the Olin Demo date as the published date. 

Second, copy the `yyyy-mm-dd-projectname.md` to the `_posts/` folder.

Finally, copy all your image files to the `assets/images/` folder.

#### Step 6: Test your post locally
Now, navigate to the website again and go to `http://127.0.0.1:4000/irl/projects.html`. You should see your post in the post list. Click on the post to make sure it works. 

(Optional) If your images do not show up, make sure you follow the steps in [How to prepare your README file](##how-to-prepare-your-readme-file) correctly.

#### Step 7: Create a pull request
Make sure you already branched off from `gh-pages` as shown in **Step 3** above, and your current branch is something like `projectname-docs`.

Then, `add`, `commit` and `push` new changes to github:
```console
$ git add .
$ git commit -m "Upload post for {cool_projet_name}"
$ git push origin {branch_name}
```
- Feel free to use a different commit message. 
- `{cool_projet_name}` is your project name, in my case `projectname`. 
- `{branch_name}` is your branch name, in my case `projectname-docs`.

Finally, go to [https://github.com/olinrobotics/irl/](https://github.com/olinrobotics/irl/) and [create a pull request](https://help.github.com/articles/creating-a-pull-request/). Make sure to add page maintainers and project leaders as reviewers.

### Option 2: ask the page maintainers to upload your post
If you aren't sure how to do Option 1, you can send the files to the page maintainers or project leaders and ask them to upload your post. Please spend time working with them closely to make sure that your post looks as expected.

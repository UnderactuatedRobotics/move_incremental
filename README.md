We are using [Vincent Driessen's branching model](http://nvie.com/posts/a-successful-git-branching-model/) for git development workflow.

In a nutshell, **develop** branch is where several members' branches merge together. Individual developer should branch off, create feature/_your_feature_ branch, develop and then finish by merging it back to develop branch.  
 
[git-flow library of git subcommands](http://jeffkreeftmeijer.com/2010/why-arent-you-using-git-flow/) makes this just as easy as your daily git commands.

Installation of git-flow on Linuxes can be found [here](https://github.com/nvie/gitflow/wiki/Linux).

It's okay that you don't want to use git-flow: just manually create your feature branch and work, don't directly work on development branch. Just don't forget to turn off fast-forward to preserve your feature branch history by typing: `git merge --no-ff develop`.

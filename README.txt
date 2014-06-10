These are the C utiliy files to manage the SoftHand

Compile the software:
Fisrt of all you need to download the qbAPI repository and be sure to have a
file tree like that:

your_workingcopy
    |
    > qbAPI
    > handadmin

Then you will need to compile the libraries. Go to qbAPI/src and type "make".
Then go to handadmin/src folder and type "make".

If everything is good, you should have a folder tree like that:

handadmin
    |
    > bin
    > conf_files
    > objs
    > src

From the handadmin folder, execute your binary files by typing:

./bin/name_of_the_bin
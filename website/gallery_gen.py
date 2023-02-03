#!/usr/bin/env python3

# gallery generator
# feb. 2, 2023
# ------------------------------
# arg 1: gallery dir on os
# arg 2: dir name on server
# arg 3: lightbox name on page

import os
from os import listdir
from os.path import isfile, join
import sys


#print ('Number of arguments:', len(sys.argv), 'arguments.')
#print ('Argument List:', str(sys.argv))
#print ('1st', str(sys.argv[1]))

GALLERY_DIR = str(sys.argv[1])
INCLUDE_CAPTIONS = False
LIGHTBOX_NAME = str(sys.argv[3])
DIR_NAME = str(sys.argv[2])


# --------------------------------------

file = open( ("%s/out.html" % GALLERY_DIR), "w+")

dir_files = os.listdir(GALLERY_DIR)
img_files = []

for f in dir_files:
	if not f.startswith('.'):
		if not f.startswith('out'):
			img_files.append(f)

#print(img_files)

# --------------------------------------

count = 0
row_opened = False

for f in img_files:
	name = f.split('.')[0]
	extension = f.split('.')[1]
	link_url = ('images/%s/%s' % (DIR_NAME, f))
	thumb_url = ('images/%s/%s-thumb.%s' % (DIR_NAME, name, extension))

	#print(link_url)
	#print(thumb_url)

	# --------------------------------------

	row_line1 = ("<div class=\"row\">\r\n")
	col_line1 = ("\t<div class=\"col-6 col-sm-3 themed-grid-col\" style=\"text-align: center;\">\r\n")
	fig_line1 = ("\t\t<figure class=\"figure\">\r\n")
	if INCLUDE_CAPTIONS:
		link_line1 = ("\t\t\t<a href=\"%s\" data-lightbox=\"%s\" data-title=\"Insert caption here\">\r\n" % (link_url, LIGHTBOX_NAME))
	else:
		link_line1 = ("\t\t\t<a href=\"%s\" data-lightbox=\"%s\">\r\n" % (link_url, LIGHTBOX_NAME))
	img_line1 = ("\t\t\t\t<img src=\"%s\" style=\"width:100%%\" class=\"rounded\" />\r\n" % thumb_url)
	caption_line = ("\t\t\t\t<figcaption class=\"figure-caption text-center\">Insert caption here</figcaption>\r\n")
	link_line2 = ("\t\t\t</a>\r\n")
	fig_line2 = ("\t\t</figure>\r\n")
	col_line2 = ("\t</div>\r\n")
	row_line2 = ("</div>\r\n\r\n\r\n")

	# --------------------------------------

	if count == 4:
		count = 0

	if count == 0:
		file.write(row_line1)
		row_opened = True

	file.write(col_line1)
	file.write(fig_line1)
	file.write(link_line1)
	file.write(img_line1)
	if INCLUDE_CAPTIONS:
		file.write(caption_line)
	file.write(link_line2)
	file.write(fig_line2)
	file.write(col_line2)

	count = count+1

	if count == 4:
		file.write(row_line2)
		row_opened = False


# can happen if count doesn't reach 4
if row_opened == True:
	file.write(row_line2)
	row_opened = False

file.close() 

# --------------------------------------


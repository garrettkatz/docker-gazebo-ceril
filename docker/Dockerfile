# Use an official gazebo as a parent image
FROM gazebo:gzserver8


MAINTAINER Garrett Katz <garrett.katz@gmail.com>


#change user to root
USER root

#Install pip
RUN apt --assume-yes update
RUN apt install --assume-yes python3-pip
RUN pip3 install --upgrade pip


# Copies any source-code you've written into the image
COPY src/ /src

#Install git and clone the CERIL repository
RUN apt --assume-yes update
RUN apt install --assume-yes git
RUN git clone https://github.com/garrettkatz/copct.git /src/


# Copies all of the pip .whl files into the image to install them
COPY docker/vendor/ /vendor

# Installs all of the .whl files
RUN set -ex \
  && pip3 install /vendor/numpy-1.17.2-cp35-cp35m-manylinux1_x86_64.whl \
  && pip3 install /vendor/scipy-1.3.1-cp35-cp35m-manylinux1_x86_64.whl \
  && pip3 install /vendor/six-1.12.0-py2.py3-none-any.whl \
  && pip3 install /vendor/Pillow-6.1.0-cp35-cp35m-manylinux1_x86_64.whl \
  && pip3 install /vendor/torch-1.2.0+cpu-cp35-cp35m-manylinux1_x86_64.whl \
  && pip3 install /vendor/torchvision-0.4.0+cpu-cp35-cp35m-manylinux1_x86_64.whl

# Cleans up by deleting the .whl files from the image
RUN set -ex \
  && rm -rf /vendor

# Starts in a bash shell and gzserver when you run the image in a container
CMD /bin/bash && gzserver

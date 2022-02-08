# See here for image tag: https://hub.docker.com/r/espressif/idf/tags 
ARG IDF_TAG="latest"
FROM espressif/idf:${IDF_TAG}

# script-library
# https://github.com/microsoft/vscode-dev-containers/tree/main/script-library

# Use the [Option] comment to specify true/false arguments that should appear in VS Code UX
#
# [Option] Install zsh
ARG INSTALL_ZSH="true"
# [Option] Upgrade OS packages to their latest versions
ARG UPGRADE_PACKAGES="false"

# Install needed packages and setup non-root user. Use a separate RUN statement to add your own dependencies.
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
COPY library-scripts/*.sh library-scripts/*.env /tmp/library-scripts/
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && /bin/bash /tmp/library-scripts/common-debian.sh "${INSTALL_ZSH}" "${USERNAME}" "${USER_UID}" "${USER_GID}" "${UPGRADE_PACKAGES}" "true" "true"\
    #
    # ****************************************************************************
    # * TODO: Add any additional OS packages you want included in the definition *
    # * here. We want to do this before cleanup to keep the "layer" small.       *
    # ****************************************************************************
    # && apt-get -y install --no-install-recommends <your-package-list-here> \
    #
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* /tmp/library-scripts


# export.sh
RUN echo '' >> /home/vscode/.profile \
    && echo '# set IDF tools path' >> /home/vscode/.profile \
    && echo 'if [ -f "$IDF_PATH/export.sh" ] ; then' >> /home/vscode/.profile \
    && echo '    . "$IDF_PATH/export.sh"' >> /home/vscode/.profile \
    && echo 'fi' >> /home/vscode/.profile \
    && echo '' >> /home/vscode/.profile

# [Optional] Uncomment this section to install additional OS packages.
# RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends <your-package-list-here>

# Install micropython and pre-build.
RUN echo '# micropython sourcecode path' >> /home/vscode/.profile \
    && echo 'export SRC_MICROPYTHON=/home/vscode/esp/micropython' >> /home/vscode/.profile \ 
    && echo '' >> /home/vscode/.profile
USER vscode
WORKDIR /home/vscode/esp
ARG MPY_TAG="master"
RUN git clone --branch=${MPY_TAG} --depth=1 https://github.com/micropython/micropython.git micropython \
    && make -C micropython/mpy-cross \
    && make -C micropython/ports/esp32 submodules

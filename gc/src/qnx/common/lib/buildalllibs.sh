#	Library build script.
#	Temporary until we get make to do this.
#	John Nagle
#	January. 2005.
#
#	Stop on first error
set +e
set +v
echo "Install all header files."
(cd cv &&   make install_headers)
(cd cvaux &&   make install_headers)
(cd ffmpeg &&   make install_headers)
# (cd gcactserver &&   make install_headers)
(cd gccalib &&   make install_headers)
(cd gccomm &&   make install_headers)
(cd gccontrol &&   make install_headers)
(cd gcmath &&   make install_headers)
(cd gcui &&   make install_headers)
(cd getoptions &&   make install_headers)
(cd usbhid &&   make install_headers)
(cd opensteer && make install_headers)

echo "Build and install all libraries"
(cd cv &&   make &&   make install)
(cd cvaux &&   make &&   make install)
(cd ffmpeg &&   make &&   make install)
# (cd gcactserver &&   make &&   make install)
(cd gccalib &&   make &&   make install)
(cd gccomm &&   make &&   make install)
(cd gccontrol &&   make &&   make install)
(cd gcmath &&   make &&   make install)
(cd gcui &&   make &&   make install)
(cd getoptions &&   make &&   make install)
(cd usbhid &&   make &&   make install)
(cd opensteer &&   make &&   make install)


echo "Build and install test programs for libraries"
# (cd check_actserver &&   make &&   make install)
(cd check_ask &&    make &&   make install)
(cd check_controller &&    make &&   make install)
(cd check_ethernet &&    make &&   make install)
(cd check_getoptions &&    make &&   make install)
(cd check_ideconsole &&    make &&   make install)
(cd check_linearreg &&    make &&   make install)
(cd check_math &&    make &&   make install)
(cd check_menu &&    make &&   make install)
# (cd check_mvp &&    make &&   make install)
(cd check_serial &&    make &&   make install)
(cd check_timedloop &&    make &&   make install)
(cd check_usbhid &&    make &&   make install)

echo "Done"

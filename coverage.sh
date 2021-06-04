#!/bin/bash
# To use this locally, go to the root of your workspace
# cd ~/<workspace-name>/
# Then run ./src/easy_manipulation_deployment/coverage.sh html
# Coverage report will be generate automatically

if [ "$1" = "ci" ]; then
  cd ~/target_ws
fi

package_name="easy_manipulation_deployment"

ignored_files="*/test/*"

# Install LCOV

if [ $(dpkg-query -W -f='${Status}' lcov 2>/dev/null | grep -c "ok installed") -eq 0 ];
then
  sudo apt-get install -y -qq lcov
fi


# Capture initial coverage info
lcov --capture --initial \
     --directory build \
     --output-file initial_coverage.info | grep -ve "^Processing"
# Capture tested coverage info
lcov --capture \
     --directory build \
     --output-file test_coverage.info | grep -ve "^Processing"
# Combine two report (exit function  when none of the records are valid)
lcov --add-tracefile initial_coverage.info \
     --add-tracefile test_coverage.info \
     --output-file coverage.info || return 0 \
  && rm initial_coverage.info test_coverage.info
# Extract repository files
lcov --extract coverage.info "$(pwd)/src/$package_name/*" \
     --output-file coverage.info | grep -ve "^Extracting"
# Filter out ignored files
lcov --remove coverage.info "$ignored_files" \
     --output-file coverage.info | grep -ve "^removing"
if [ "$1" = "ci" ]; then
  # Some sed magic to remove identifiable absolute path
  sed -i "s~$(pwd)/src/$package_name/~~g" coverage.info
  lcov --list coverage.info

  cd -
  cp -r ~/target_ws/coverage.info .

elif [ "$1" = "html" ]; then
  genhtml coverage.info -o coverage
fi

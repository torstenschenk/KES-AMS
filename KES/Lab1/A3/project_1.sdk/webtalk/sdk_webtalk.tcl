webtalk_init -webtalk_dir C:\\Users\\Misca\\Documents\\KES-AMS\\KES\\Lab1\\A3\\project_1.sdk\\webtalk
webtalk_register_client -client project
webtalk_add_data -client project -key date_generated -value "Mon Nov 19 23:44:23 2018" -context "software_version_and_target_device"
webtalk_add_data -client project -key product_version -value "SDK v2016.2" -context "software_version_and_target_device"
webtalk_add_data -client project -key build_version -value "2016.2" -context "software_version_and_target_device"
webtalk_add_data -client project -key os_platform -value "amd64" -context "software_version_and_target_device"
webtalk_add_data -client project -key registration_id -value "" -context "software_version_and_target_device"
webtalk_add_data -client project -key tool_flow -value "SDK" -context "software_version_and_target_device"
webtalk_add_data -client project -key beta -value "false" -context "software_version_and_target_device"
webtalk_add_data -client project -key route_design -value "NA" -context "software_version_and_target_device"
webtalk_add_data -client project -key target_family -value "NA" -context "software_version_and_target_device"
webtalk_add_data -client project -key target_device -value "NA" -context "software_version_and_target_device"
webtalk_add_data -client project -key target_package -value "NA" -context "software_version_and_target_device"
webtalk_add_data -client project -key target_speed -value "NA" -context "software_version_and_target_device"
webtalk_add_data -client project -key random_id -value "kprduv386q7nb4nodp4jamr2ud" -context "software_version_and_target_device"
webtalk_add_data -client project -key project_id -value "2016.2_6" -context "software_version_and_target_device"
webtalk_add_data -client project -key project_iteration -value "6" -context "software_version_and_target_device"
webtalk_add_data -client project -key os_name -value "" -context "user_environment"
webtalk_add_data -client project -key os_release -value "" -context "user_environment"
webtalk_add_data -client project -key cpu_name -value "" -context "user_environment"
webtalk_add_data -client project -key cpu_speed -value "" -context "user_environment"
webtalk_add_data -client project -key total_processors -value "" -context "user_environment"
webtalk_add_data -client project -key system_ram -value "" -context "user_environment"
webtalk_register_client -client sdk
webtalk_add_data -client sdk -key uid -value "1542667215806" -context "sdk\\\\hardware/1542667215806"
webtalk_add_data -client sdk -key isZynq -value "true" -context "sdk\\\\hardware/1542667215806"
webtalk_add_data -client sdk -key Processors -value "2" -context "sdk\\\\hardware/1542667215806"
webtalk_add_data -client sdk -key VivadoVersion -value "2016.2" -context "sdk\\\\hardware/1542667215806"
webtalk_add_data -client sdk -key Arch -value "zynq" -context "sdk\\\\hardware/1542667215806"
webtalk_add_data -client sdk -key Device -value "7z020" -context "sdk\\\\hardware/1542667215806"
webtalk_add_data -client sdk -key IsHandoff -value "true" -context "sdk\\\\hardware/1542667215806"
webtalk_transmit -clientid 1820567481 -regid "" -xml C:\\Users\\Misca\\Documents\\KES-AMS\\KES\\Lab1\\A3\\project_1.sdk\\webtalk\\usage_statistics_ext_sdk.xml -html C:\\Users\\Misca\\Documents\\KES-AMS\\KES\\Lab1\\A3\\project_1.sdk\\webtalk\\usage_statistics_ext_sdk.html -wdm C:\\Users\\Misca\\Documents\\KES-AMS\\KES\\Lab1\\A3\\project_1.sdk\\webtalk\\sdk_webtalk.wdm -intro "<H3>SDK Usage Report</H3><BR>"
webtalk_terminate

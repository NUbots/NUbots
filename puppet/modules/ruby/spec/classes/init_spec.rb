require 'spec_helper'
describe 'ruby', :type => :class do

  describe 'when called with no parameters on redhat' do
    let (:facts) { {  :osfamily => 'Redhat',
                      :path => '/usr/local/bin:/usr/bin:/bin' } }
    it {
      should contain_package('ruby').with({
        'ensure'  => 'installed',
        'name'    => 'ruby',
      })
      should contain_package('rubygems').with({
        'ensure'  => 'installed',
        'require' => 'Package[ruby]',
      })
      should contain_package('rubygems-update').with({
        'ensure'    => 'installed',
        'require'   => 'Package[rubygems]',
        'provider'  => 'gem',
      })
      should contain_exec('ruby::update_rubygems').with({
        'path'        => '/usr/local/bin:/usr/bin:/bin',
        'command'     => 'update_rubygems',
        'subscribe'   => 'Package[rubygems-update]',
        'refreshonly' => true,
      })
    }
  end

  describe 'when called with no parameters on debian' do
    let (:facts) { {  :osfamily => 'Debian',
                      :path     => '/usr/local/bin:/usr/bin:/bin' } }
    it {
      should contain_package('ruby').with({
        'ensure'  => 'installed',
        'name'    => 'ruby',
      })
      should contain_package('rubygems').with({
        'ensure'  => 'installed',
        'require' => 'Package[ruby]',
      })
      should_not contain_package('rubygems-update')
      should_not contain_exec('ruby::update_rubygems')
    }
  end

  describe 'when called with custom rubygems version on redhat' do
    let (:facts) { {   :osfamily  => 'Redhat',
                       :path      => '/usr/local/bin:/usr/bin:/bin' } }
    let (:params) { {  :gems_version => '1.8.7' } }
    it {
      should contain_package('ruby').with({
        'ensure'  => 'installed',
        'name'    => 'ruby',
      })
      should contain_package('rubygems').with({
        'ensure'  => 'installed',
        'require' => 'Package[ruby]',
      })
      should contain_package('rubygems-update').with({
        'ensure'    => '1.8.7',
        'require'   => 'Package[rubygems]',
        'provider'  => 'gem',
      })
      should contain_exec('ruby::update_rubygems').with({
        'path'        => '/usr/local/bin:/usr/bin:/bin',
        'command'     => 'update_rubygems',
        'subscribe'   => 'Package[rubygems-update]',
        'refreshonly' => true,
      })
    }
  end

  describe 'when called with custom ruby package name' do
    let (:facts) { { :osfamily => 'Debian',
                     :path     => '/usr/local/bin:/usr/bin:/bin' } }
    let (:params) { {  :ruby_package  => 'ruby1.9' } }
    it {
      should contain_package('ruby').with({
        'ensure'  => 'installed',
        'name'    => 'ruby1.9',
      })
      should contain_package('rubygems').with({
        'ensure'  => 'installed',
        'require' => 'Package[ruby]',
      })
      should_not contain_package('rubygems-update')
      should_not contain_exec('ruby::update_rubygems')
    }
  end

  describe 'when called with custom rubygems package name' do
    let (:facts) { { :osfamily => 'Debian',
                     :path     => '/usr/local/bin:/usr/bin:/bin' } }
    let (:params) { { :rubygems_package  => 'rubygems1.9.1' } }
    it {
      should contain_package('ruby').with({
        'ensure'  => 'installed',
      })
      should contain_package('rubygems').with({
        'name'    => 'rubygems1.9.1',
        'ensure'  => 'installed',
        'require' => 'Package[ruby]',
      })
      should_not contain_package('rubygems-update')
      should_not contain_exec('ruby::update_rubygems')
    }
  end


  describe 'when called with custom rubygems and ruby versions on redhat' do
    let (:facts) { {  :osfamily => 'Redhat',
                      :path     => '/usr/local/bin:/usr/bin:/bin' } }
    let (:params) { {   :gems_version => '1.8.6',
                        :version      => '1.8.7', } }
    it {
      should contain_package('ruby').with({
        'ensure'  => '1.8.7',
        'name'    => 'ruby',
      })
      should contain_package('rubygems').with({
        'ensure'  => 'installed',
        'require' => 'Package[ruby]',
      })
      should contain_package('rubygems-update').with({
        'ensure'    => '1.8.6',
        'require'   => 'Package[rubygems]',
        'provider'  => 'gem',
      })
      should contain_exec('ruby::update_rubygems').with({
        'path'        => '/usr/local/bin:/usr/bin:/bin',
        'command'     => 'update_rubygems',
        'subscribe'   => 'Package[rubygems-update]',
        'refreshonly' => true,
      })
    }
  end

  describe 'when called with custom rubygems and ruby versions on debian' do
    let (:facts) { {  :osfamily => 'Debian',
                      :path     => '/usr/local/bin:/usr/bin:/bin' } }
    let (:params) { {   :gems_version => '1.8.6',
                        :version      => '1.8.7', } }
    it {
      should contain_package('ruby').with({
        'ensure'  => '1.8.7',
        'name'    => 'ruby',
      })
      should contain_package('rubygems').with({
        'ensure'  => '1.8.6',
        'require' => 'Package[ruby]',
      })
      should_not contain_package('rubygems-update')
      should_not contain_exec('ruby::update_rubygems')
    }
  end

  describe 'when called with custom rubygems version and no rubygems_update on debian' do
    let (:facts) { {    :osfamily => 'Debian',
                        :path => '/usr/local/bin:/usr/bin:/bin' } }
    let (:params) { {   :gems_version     => '1.8.7',
                        :rubygems_update  => false, } }
    it {
      should contain_package('ruby').with({
        'ensure'  => 'installed',
        'name'    => 'ruby',
      })
      should contain_package('rubygems').with({
        'ensure'  => '1.8.7',
        'require' => 'Package[ruby]',
      })
      should_not contain_package('rubygems-update')
      should_not contain_exec('ruby::update_rubygems')
    }
  end

  describe 'when called with custom rubygems version and no rubygems_update on redhat' do
    let (:facts) { {    :osfamily => 'Redhat',
                        :path => '/usr/local/bin:/usr/bin:/bin' } }
    let (:params) { {   :gems_version     => '1.8.7',
                        :rubygems_update  => false, } }
    it {
      should contain_package('ruby').with({
        'ensure'  => 'installed',
        'name'    => 'ruby',
      })
      should contain_package('rubygems').with({
        'ensure'  => '1.8.7',
        'require' => 'Package[ruby]',
      })
      should_not contain_package('rubygems-update')
      should_not contain_exec('ruby::update_rubygems')
    }
  end

end

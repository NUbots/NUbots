module Puppet::Parser::Functions
    newfunction(:lsb_release, :type => :rvalue) do |args|
        `lsb_release -cs`.chomp
    end
end

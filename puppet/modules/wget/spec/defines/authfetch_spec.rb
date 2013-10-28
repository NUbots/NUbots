require 'spec_helper'

describe 'wget::authfetch' do
  let(:title) { 'authtest' }
  let(:params) {{
    :source      => 'http://localhost/source',
    :destination => '/tmp/dest',
    :user        => 'myuser',
    :password    => 'mypassword',
  }}

  context "with default params" do
    it { should contain_exec('wget-authtest').with({
      'command'     => "wget --no-verbose --user=myuser --output-document='/tmp/dest' 'http://localhost/source'",
      'environment' => 'WGETRC=/tmp/wgetrc-authtest'
      })
    }
    it { should contain_file('/tmp/wgetrc-authtest').with_content('password=mypassword') }
  end

  context "with user" do
    let(:params) { super().merge({
      :execuser => 'testuser',
    })}

    it { should contain_exec('wget-authtest').with({
      'command' => "wget --no-verbose --user=myuser --output-document='/tmp/dest' 'http://localhost/source'",
      'user'    => 'testuser'
    }) }
  end
end

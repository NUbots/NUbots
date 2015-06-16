require 'spec_helper'

describe 'wget::fetch' do
  let(:title) { 'test' }

  let(:params) {{
    :source      => 'http://localhost/source',
    :destination => '/tmp/dest',
  }}

  context "with default params" do
    it { should contain_exec('wget-test').with_command("wget --no-verbose --output-document='/tmp/dest' 'http://localhost/source'") }
  end

  context "with user" do
    let(:params) { super().merge({
      :execuser => 'testuser',
    })}

    it { should contain_exec('wget-test').with({
      'command' => "wget --no-verbose --output-document='/tmp/dest' 'http://localhost/source'",
      'user'    => 'testuser'
    }) }
  end
end

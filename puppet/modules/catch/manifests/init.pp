class catch {
  wget::fetch { 'catch.hpp':
    destination => '/usr/local/include/catch.hpp',
    # source => 'https://raw.github.com/philsquared/Catch/5ecb72b9bb65cd8fed2aec4da23a3bc21bbccd74/single_include/catch.hpp',
    source => 'https://raw.githubusercontent.com/philsquared/Catch/544bf33e73e10ad074a410f2cb25c310351ebfac/single_include/catch.hpp',
  }
}
